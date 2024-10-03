#include "unitree_motor_controller/motor_controller.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

MotorController::MotorController()
: Node("motor_controller"), serial_("/dev/ttyUSB0") {

    // Create services for calibration
    start_calibration_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_calibration", std::bind(&MotorController::startCalibration, this, std::placeholders::_1, std::placeholders::_2));

    // Create a subscriber to the 'joint_position' topic
    joint_position_sub_ = this->create_subscription<bdx_msgs::msg::JointPositionTarget>(
        "joint_position_target", 10, std::bind(&MotorController::jointPositionCallback, this, std::placeholders::_1));
    
    // Declare and get the parameter for publish rate
    this->declare_parameter<int>("joint_state_publish_rate", 100);
    this->declare_parameter<int>("motor_command_publish_rate", 100);
    this->get_parameter("joint_state_publish_rate", joint_state_publish_rate_);
    this->get_parameter("motor_command_publish_rate", motor_command_publish_rate_);

    // Declare and get the parameter for motor type
    this->declare_parameter<std::string>("motor_type", "GO_M8010_6");
    this->get_parameter("motor_type", motor_type_);

    // Declare the paramter file
    this->declare_parameter<std::string>("config_file", "/home/toro/motor_limits.yaml");
    this->get_parameter("config_file", config_file_);
    RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file_.c_str());

    // Set motor type logic
    if (motor_type_ == "A1") {
        selected_motor_type_ = MotorType::A1;
    } else if (motor_type_ == "GO_M8010_6") {
        selected_motor_type_ = MotorType::GO_M8010_6;
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid motor type: %s", motor_type_.c_str());
    }

    // Declare and get the parameter for motor ID
    this->declare_parameter<int>("motor_id", 0);
    this->get_parameter("motor_id", motor_id_);
    // Declare and get the parameter for joint name
    this->declare_parameter<std::string>("joint_name", "unitree_motor_joint");
    this->get_parameter("joint_name", joint_name_);

    // Create a publisher for the 'joint_states' topic
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(joint_name_ + "_status", 10);

    // Load motor limits from YAML config file
    calibrated_ = loadMotorLimits();

    // Timer to publish current motor state periodically
    joint_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / joint_state_publish_rate_),
        std::bind(&MotorController::publishJointState, this));

    // Timer to update motor position at a fixed rate
    motor_command_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / motor_command_publish_rate_),
        std::bind(&MotorController::updateMotorPosition, this));

    // Initialize motor parameters
    gear_ratio_ = queryGearRatio(selected_motor_type_);

    RCLCPP_INFO(this->get_logger(), "Gear ratio: %.2f", gear_ratio_);

    // Set rotor_kp_ and rotor_kd_ based on the motor type
    if (motor_type_ == "A1") {
        rotor_kp_ = (output_kp_ / (gear_ratio_ * gear_ratio_)) / 26.07;
        rotor_kd_ = (output_kd_ / (gear_ratio_ * gear_ratio_)) * 100.0;
    } else if (motor_type_ == "GO_M8010_6") {
        rotor_kp_ = (output_kp_ / (gear_ratio_ * gear_ratio_));
        rotor_kd_ = (output_kd_ / (gear_ratio_ * gear_ratio_));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Invalid motor type: %s", motor_type_.c_str());
    }

    // Set the initial movement speed to the maximum speed
    movement_speed_ = max_speed_;

    motor_cmd_.motorType = selected_motor_type_;
    motor_data_.motorType = selected_motor_type_;

    sendRecvMotorCmd(motor_id_, queryMotorMode(selected_motor_type_, MotorMode::BRAKE), 0, 0, 0, 0, 0);

    // Record the initial motor position
    initial_position_ = current_position_;
    target_position_ = current_position_;
    RCLCPP_INFO(this->get_logger(), "Initial motor position set to %.2f degrees as the reference.", initial_position_);

    if (!calibrated_) {
        RCLCPP_WARN(this->get_logger(), "Motor calibration required. Please perform the motor routine.");
    }
}

MotorController::~MotorController() {
    RCLCPP_INFO(this->get_logger(), "Setting motor to BRAKE mode before shutdown.");
    sendRecvMotorCmd(motor_id_, queryMotorMode(selected_motor_type_, MotorMode::BRAKE), 0, 0, 0, 0, 0);
}

// wrapper function for serial_.sendRecv that sends the motor command and updates the current position, velocity and effort member variables
void MotorController::sendRecvMotorCmd(unsigned short id, unsigned short mode, float tau, float dq, float q, float kp, float kd) {
    motor_cmd_.id = id;
    motor_cmd_.mode = mode;
    motor_cmd_.tau = tau;
    motor_cmd_.dq = dq;
    motor_cmd_.q = q;
    motor_cmd_.kp = kp;
    motor_cmd_.kd = kd;
    serial_.sendRecv(&motor_cmd_, &motor_data_);

    current_position_ = motor_data_.q / gear_ratio_ * (180 / M_PI);
    current_velocity_ = motor_data_.dq / gear_ratio_ * (180 / M_PI);
    current_effort_ = motor_data_.tau;
}

void MotorController::startCalibration(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        if (!is_calibrating_) {
            is_calibrating_ = true;
            min_position_ = std::numeric_limits<float>::max();
            max_position_ = std::numeric_limits<float>::lowest();
            sendRecvMotorCmd(motor_id_, queryMotorMode(selected_motor_type_, MotorMode::FOC), 0, 0, 0, 0, 0);
            calibration_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100), std::bind(&MotorController::calibrationLoop, this));
            response->success = true;
            response->message = "Calibration started.";
        } else {
            response->success = false;
            response->message = "Calibration is already in progress.";
        }
    } else {
        if (is_calibrating_) {
            is_calibrating_ = false;
            calibration_timer_->cancel();
            response->success = true;
            response->message = "Calibration stopped.";
            bool saved = saveMotorLimits();
            if (saved) {
                RCLCPP_INFO(this->get_logger(), "Motor limits saved to %s", config_file_.c_str());
                calibrated_ = true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to save motor limits to %s", config_file_.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "Calibration completed. Min position: %.2f, Max position: %.2f", min_position_, max_position_);
        } else {
            response->success = false;
            response->message = "Calibration is not in progress.";
        }
    }
}

void MotorController::calibrationLoop() {

    sendRecvMotorCmd(motor_id_, queryMotorMode(selected_motor_type_, MotorMode::FOC), 0, 0, 0, 0, 0);

    if (current_position_ < min_position_) {
        min_position_ = current_position_;
        RCLCPP_INFO(this->get_logger(), "New Min position: %.2f", min_position_);
    }
    if (current_position_ > max_position_) {
        max_position_ = current_position_;
        RCLCPP_INFO(this->get_logger(), "New Max position: %.2f", max_position_);
    }
}

bool MotorController::loadMotorLimits() {
    RCLCPP_INFO(this->get_logger(), "Loading motor limits from %s", config_file_.c_str());

    try {
        YAML::Node config = YAML::LoadFile(config_file_);
        min_position_ = config["min_position"].as<float>();
        max_position_ = config["max_position"].as<float>();
        float min_adjusted_postion = min_position_ - initial_position_;
        float max_adjusted_postion = max_position_ - initial_position_;

        RCLCPP_INFO(this->get_logger(), "Loaded motor limits from config: min=%.2f, max=%.2f", min_adjusted_postion, max_adjusted_postion);
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load motor limits: %s", e.what());
        return false;
    }

    return true;
}

bool MotorController::saveMotorLimits() {
    YAML::Node config;
    config["min_position"] = min_position_;
    config["max_position"] = max_position_;
    float min_adjusted_postion = min_position_ - initial_position_;
    float max_adjusted_postion = max_position_ - initial_position_;

    std::ofstream fout(config_file_);
    fout << config;
    RCLCPP_INFO(this->get_logger(), "Motor limits saved to %s: min=%.2f, max=%.2f", config_file_.c_str(), min_adjusted_postion, max_adjusted_postion);
    return true;
}

void MotorController::jointPositionCallback(const bdx_msgs::msg::JointPositionTarget::SharedPtr msg) {
    // Update the target position relative to the initial position
    target_position_ = initial_position_ + msg->target_position;
    movement_speed_ = msg->movement_speed;
    RCLCPP_INFO(this->get_logger(), "Received new target position: %.2f degrees, target speed: %.2f", target_position_, movement_speed_);
}

void MotorController::updateMotorPosition() {
    // Motor Calibratiion is active, do not update motor position
    if (is_calibrating_) {
        return;
    }

    if (!calibrated_) {
        RCLCPP_DEBUG(this->get_logger(), "Motor calibration required. Aborting motor control.");
        sendRecvMotorCmd(motor_id_, queryMotorMode(selected_motor_type_, MotorMode::FOC), 0, 0, 0, 0, 0);
        return;
    }

    // Calculate the distance to the target
    float distance_to_target = target_position_ - current_position_;

    // Adjust the speed based on the distance to the target
    float speed = movement_speed_;
    if (fabs(distance_to_target) < ramp_distance_) {
        speed = movement_speed_ * (fabs(distance_to_target) / ramp_distance_);
        if (speed < min_speed_) speed = min_speed_;
    }

    // Calculate the time interval in seconds from the publish rate
    float time_interval = 1.0 / motor_command_publish_rate_;

    // Calculate the incremental step based on the desired speed
    float increment = (speed * time_interval) * (distance_to_target > 0 ? 1 : -1);

   // Calculate the new position based on the speed
    float new_position = current_position_ + increment;

    // Ensure the new position does not exceed the limits
    if (new_position < min_position_) {
        new_position = min_position_;
    } else if (new_position > max_position_) {
        new_position = max_position_;
    }
    // Update current position
    //current_position_ = new_position;

    // Convert the current position to rotor angle
    float rotor_angle_d = (new_position * (M_PI / 180)) * gear_ratio_;

    // Send the command to the motor
    sendRecvMotorCmd(motor_id_, queryMotorMode(selected_motor_type_, MotorMode::FOC), 0, 0, rotor_angle_d, rotor_kp_, rotor_kd_);

    RCLCPP_DEBUG(this->get_logger(), "Current position: %.2f degrees, current speed: %.2f, movement speed: %.2f", current_position_, movement_speed_, current_velocity_);
}

void MotorController::publishJointState() {
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.push_back(joint_name_);

    // Calculate current angle relative to the initial position
    joint_state_msg.position.push_back((current_position_ - initial_position_)* (M_PI / 180));
    joint_state_msg.velocity.push_back(current_velocity_);
    joint_state_msg.effort.push_back(current_effort_);

    joint_state_pub_->publish(joint_state_msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}

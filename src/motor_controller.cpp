#include "unitree_motor_controller/motor_controller.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

MotorController::MotorController()
: Node("motor_controller"), serial_("/dev/ttyUSB0") {
    // Load motor limits from YAML config file
    bool calibrated = loadMotorLimits();

    // Create a subscriber to the 'joint_position' topic
    joint_position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        "joint_position", 10, std::bind(&MotorController::jointPositionCallback, this, std::placeholders::_1));
    
    // Create a publisher for the 'joint_states' topic
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

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
    motor_cmd_.mode = queryMotorMode(selected_motor_type_, MotorMode::BRAKE);
    motor_cmd_.id = motor_id_;
    serial_.sendRecv(&motor_cmd_, &motor_data_);

    // Record the initial motor position
    initial_position_ = motor_data_.q / gear_ratio_ * (180 / M_PI);
    current_position_ = initial_position_;
    target_position_ = initial_position_;
    RCLCPP_INFO(this->get_logger(), "Initial motor position set to %.2f degrees as the reference.", initial_position_);

    if (!calibrated) {
        RCLCPP_WARN(this->get_logger(), "Motor calibration required. Please move motor to the limits.");
        performMotorCalibration();
    }
}

MotorController::~MotorController() {
    RCLCPP_INFO(this->get_logger(), "Setting motor to BRAKE mode before shutdown.");
    motor_cmd_.mode = queryMotorMode(selected_motor_type_, MotorMode::BRAKE);
    serial_.sendRecv(&motor_cmd_, &motor_data_);
}

bool MotorController::loadMotorLimits() {
    std::string config_file;
    this->get_parameter("config_file", config_file);

    try {
        YAML::Node config = YAML::LoadFile(config_file);
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

bool MotorController::performMotorCalibration() {
    RCLCPP_INFO(this->get_logger(), "Move the motor to the first limit (min or max), waiting for position change...");
    motor_cmd_.mode = queryMotorMode(selected_motor_type_, MotorMode::BRAKE);
    serial_.sendRecv(&motor_cmd_, &motor_data_);

    bool min_position_detected = false;
    bool max_position_detected = false;

    float start_position = motor_data_.q / gear_ratio_ * (180 / M_PI);
    float last_position = start_position;
    auto start_time = this->now();

    // Wait for motor position to stop changing
    while (rclcpp::ok()) {
        serial_.sendRecv(&motor_cmd_, &motor_data_);
        current_position_ = motor_data_.q / gear_ratio_ * (180 / M_PI);

        if ((fabs(current_position_ - last_position) < 0.01) && (current_position_ != initial_position_)) {
            if ((this->now() - start_time).seconds() > 10) {
                min_position_ = current_position_;
                min_position_detected = true;
                RCLCPP_INFO(this->get_logger(), "First limit (min or max) detected at %.2f degrees.", min_position_);
                break;
            }
        } else {
            start_time = this->now();
        }
        last_position = current_position_;
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(this->get_logger(), "Now move the motor to the other limit (opposite end)...");

    // Similar logic for detecting the second limit
    start_time = this->now();
    last_position = motor_data_.q / gear_ratio_ * (180 / M_PI);

    while (rclcpp::ok()) {
        serial_.sendRecv(&motor_cmd_, &motor_data_);
        current_position_ = motor_data_.q / gear_ratio_ * (180 / M_PI);

        if ((fabs(current_position_ - last_position) < 0.01)  && (current_position_ != initial_position_) && (current_position_ != min_position_)) {
            if ((this->now() - start_time).seconds() > 10) {
                max_position_ = current_position_;
                max_position_detected = true;
                RCLCPP_INFO(this->get_logger(), "Second limit (min or max) detected at %.2f degrees.", max_position_);
                break;
            }
        } else {
            start_time = this->now();
        }
        last_position = current_position_;
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // set movement speed to 100 to slow down movement after calibaration is done
    movement_speed_ = 100;

    // Only call saveMotorLimits if both positions are detected
    if (min_position_detected && max_position_detected) {
        return saveMotorLimits();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Cannot save motor limits. Both min and max positions must be detected.");
        return false;
    }
}

bool MotorController::saveMotorLimits() {
    std::string config_file;
    this->get_parameter("config_file", config_file);
    
    YAML::Node config;
    config["min_position"] = min_position_;
    config["max_position"] = max_position_;
    float min_adjusted_postion = min_position_ - initial_position_;
    float max_adjusted_postion = max_position_ - initial_position_;

    std::ofstream fout(config_file);
    fout << config;
    RCLCPP_INFO(this->get_logger(), "Motor limits saved to %s: min=%.2f, max=%.2f", config_file.c_str(), min_adjusted_postion, max_adjusted_postion);
    return true;
}

void MotorController::jointPositionCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Update the target position relative to the initial position
    target_position_ = initial_position_ + msg->data;
    movement_speed_ = max_speed_;
    RCLCPP_INFO(this->get_logger(), "Received new target position: %.2f degrees, target speed: %.2f", target_position_, movement_speed_);
}

void MotorController::updateMotorPosition() {
    if (min_position_ == 0.0 || max_position_ == 0.0) {
        RCLCPP_WARN(this->get_logger(), "Motor calibration required. Aborting motor control.");
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
    motor_cmd_.motorType = selected_motor_type_;
    motor_cmd_.mode = queryMotorMode(selected_motor_type_, MotorMode::FOC); // Resume normal operation
    motor_cmd_.q = rotor_angle_d;
    motor_cmd_.kp = rotor_kp_;
    motor_cmd_.kd = rotor_kd_;
    serial_.sendRecv(&motor_cmd_, &motor_data_);

    current_position_ = motor_data_.q / gear_ratio_ * (180 / M_PI);
    current_velocity_ = motor_data_.dq / gear_ratio_ * (180 / M_PI);
    current_effort_ = motor_data_.tau;
    RCLCPP_INFO(this->get_logger(), "Current position: %.2f degrees, current speed: %.2f, movement speed: %.2f", current_position_, movement_speed_, current_velocity_);
}

void MotorController::publishJointState() {
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.push_back(joint_name_);

    // Calculate current angle relative to the initial position
    joint_state_msg.position.push_back(current_position_ - initial_position_);
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

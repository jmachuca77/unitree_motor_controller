#include "unitree_motor_controller/motor_controller.hpp"
#include <yaml-cpp/yaml.h>
#include <fstream>

MotorController::MotorController()
: Node("motor_controller"), serial_("/dev/ttyUSB0") {
    // Declare and get the parameter for publish rate
    this->declare_parameter<int>("joint_state_publish_rate", 100);
    this->declare_parameter<int>("motor_command_publish_rate", 100);
    this->get_parameter("joint_state_publish_rate", joint_state_publish_rate_);
    this->get_parameter("motor_command_publish_rate", motor_command_publish_rate_);

    // Declare the config file parameter
    this->declare_parameter<std::string>("config_file", "/home/toro/motor_limits.yaml");
    this->get_parameter("config_file", config_file_);
    RCLCPP_INFO(this->get_logger(), "Config file: %s", config_file_.c_str());

    // Load motor configurations from the config file
    YAML::Node config = YAML::LoadFile(config_file_);
    YAML::Node motors_config = config["motors"];

    for (std::size_t i = 0; i < motors_config.size(); ++i) {
        MotorParameters motor;
        motor.motor_id = motors_config[i]["motor_id"].as<int>();
        motor.joint_name = motors_config[i]["joint_name"].as<std::string>();
        motor.motor_type = motors_config[i]["motor_type"].as<std::string>();
        motor.output_kp = motors_config[i]["output_kp"].as<float>();
        motor.output_kd = motors_config[i]["output_kd"].as<float>();
        motor.movement_speed = motors_config[i]["movement_speed"].as<float>();

        // Set motor type logic
        if (motor.motor_type == "A1") {
            motor.selected_motor_type = MotorType::A1;
        } else if (motor.motor_type == "GO_M8010_6") {
            motor.selected_motor_type = MotorType::GO_M8010_6;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid motor type: %s", motor.motor_type.c_str());
        }

        // Initialize motor parameters
        motor.gear_ratio = queryGearRatio(motor.selected_motor_type);

        // Set rotor_kp_ and rotor_kd_ based on the motor type
        if (motor.motor_type == "A1") {
            motor.rotor_kp = (motor.output_kp / (motor.gear_ratio * motor.gear_ratio)) / 26.07;
            motor.rotor_kd = (motor.output_kd / (motor.gear_ratio * motor.gear_ratio)) * 100.0;
        } else if (motor.motor_type == "GO_M8010_6") {
            motor.rotor_kp = (motor.output_kp / (motor.gear_ratio * motor.gear_ratio));
            motor.rotor_kd = (motor.output_kd / (motor.gear_ratio * motor.gear_ratio));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Invalid motor type: %s", motor.motor_type.c_str());
        }

        motor.max_speed_ = 300; // Default values, can be customized
        motor.min_speed_ = 5.0;
        motor.ramp_distance_ = 10.0;

        motor.calibrated = false;
        motor.is_calibrating = false;
        motor.max_position = std::numeric_limits<float>::lowest();
        motor.min_position = std::numeric_limits<float>::max();

        motor.motor_cmd.motorType = motor.selected_motor_type;
        motor.motor_data.motorType = motor.selected_motor_type;

        // Create a subscriber to the 'joint_position_target' topic for each motor
        motor.joint_position_sub_ = this->create_subscription<bdx_msgs::msg::JointPositionTarget>(
            "joint_position_target_" + std::to_string(motor.motor_id), 10,
            [this, i](const bdx_msgs::msg::JointPositionTarget::SharedPtr msg) {
                this->jointPositionCallback(msg, i);
            });

        // Create a publisher for the 'joint_states' topic for each motor
        motor.joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            motor.joint_name + "_status", 10);

        // Initialize positions
        motor.initial_position = 0.0;
        motor.current_position = 0.0;
        motor.target_position = 0.0;

        // Initialize motor
        sendRecvMotorCmd(motor, queryMotorMode(motor.selected_motor_type, MotorMode::BRAKE), 0, 0, 0, 0, 0);

        motors_.push_back(motor);
    }

    // Create services for calibration
    start_calibration_service_ = this->create_service<std_srvs::srv::SetBool>(
        "start_calibration", std::bind(&MotorController::startCalibration, this, std::placeholders::_1, std::placeholders::_2));

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

    if (!calibrated_) {
        RCLCPP_WARN(this->get_logger(), "Motor calibration required. Please perform the motor routine.");
    }
}

MotorController::~MotorController() {
    RCLCPP_INFO(this->get_logger(), "Setting motors to BRAKE mode before shutdown.");
    for (auto& motor : motors_) {
        sendRecvMotorCmd(motor, queryMotorMode(motor.selected_motor_type, MotorMode::BRAKE), 0, 0, 0, 0, 0);
    }
}

void MotorController::sendRecvMotorCmd(MotorParameters& motor, unsigned short mode, float tau, float dq, float q, float kp, float kd) {
    motor.motor_cmd.id = motor.motor_id;
    motor.motor_cmd.mode = mode;
    motor.motor_cmd.tau = tau;
    motor.motor_cmd.dq = dq;
    motor.motor_cmd.q = q;
    motor.motor_cmd.kp = kp;
    motor.motor_cmd.kd = kd;
    serial_.sendRecv(&motor.motor_cmd, &motor.motor_data);

    motor.current_position = motor.motor_data.q / motor.gear_ratio * (180 / M_PI);
    motor.current_velocity = motor.motor_data.dq / motor.gear_ratio * (180 / M_PI);
    motor.current_effort = motor.motor_data.tau;
}

void MotorController::startCalibration(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    if (request->data) {
        if (!is_calibrating_) {
            is_calibrating_ = true;
            for (auto& motor : motors_) {
                motor.min_position = std::numeric_limits<float>::max();
                motor.max_position = std::numeric_limits<float>::lowest();
                sendRecvMotorCmd(motor, queryMotorMode(motor.selected_motor_type, MotorMode::FOC), 0, 0, 0, 0, 0);
            }
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
            for (auto& motor : motors_) {
                RCLCPP_INFO(this->get_logger(), "Calibration completed for motor %d. Min position: %.2f, Max position: %.2f",
                            motor.motor_id, motor.min_position, motor.max_position);
            }
        } else {
            response->success = false;
            response->message = "Calibration is not in progress.";
        }
    }
}

void MotorController::calibrationLoop() {
    for (auto& motor : motors_) {
        sendRecvMotorCmd(motor, queryMotorMode(motor.selected_motor_type, MotorMode::FOC), 0, 0, 0, 0, 0);

        if (motor.current_position < motor.min_position) {
            motor.min_position = motor.current_position;
            RCLCPP_INFO(this->get_logger(), "Motor %d: New Min position: %.2f", motor.motor_id, motor.min_position);
        }
        if (motor.current_position > motor.max_position) {
            motor.max_position = motor.current_position;
            RCLCPP_INFO(this->get_logger(), "Motor %d: New Max position: %.2f", motor.motor_id, motor.max_position);
        }
    }
}

bool MotorController::loadMotorLimits() {
    RCLCPP_INFO(this->get_logger(), "Loading motor limits from %s", config_file_.c_str());

    try {
        YAML::Node config = YAML::LoadFile(config_file_);
        YAML::Node limits = config["motor_limits"];
        for (auto& motor : motors_) {
            int id = motor.motor_id;
            motor.min_position = limits[id]["min_position"].as<float>();
            motor.max_position = limits[id]["max_position"].as<float>();
            float min_adjusted_position = motor.min_position - motor.initial_position;
            float max_adjusted_position = motor.max_position - motor.initial_position;

            RCLCPP_INFO(this->get_logger(), "Loaded motor %d limits: min=%.2f, max=%.2f",
                        id, min_adjusted_position, max_adjusted_position);
            motor.calibrated = true;
        }
    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load motor limits: %s", e.what());
        return false;
    }

    return true;
}

bool MotorController::saveMotorLimits() {
    YAML::Node config;
    YAML::Node limits;

    for (auto& motor : motors_) {
        int id = motor.motor_id;
        limits[id]["min_position"] = motor.min_position;
        limits[id]["max_position"] = motor.max_position;
        float min_adjusted_position = motor.min_position - motor.initial_position;
        float max_adjusted_position = motor.max_position - motor.initial_position;

        RCLCPP_INFO(this->get_logger(), "Motor %d limits: min=%.2f, max=%.2f",
                    id, min_adjusted_position, max_adjusted_position);
    }

    config["motor_limits"] = limits;

    std::ofstream fout(config_file_);
    fout << config;
    RCLCPP_INFO(this->get_logger(), "Motor limits saved to %s", config_file_.c_str());
    return true;
}

void MotorController::jointPositionCallback(const bdx_msgs::msg::JointPositionTarget::SharedPtr msg, int motor_index) {
    // Update the target position relative to the initial position
    auto& motor = motors_[motor_index];
    motor.target_position = motor.initial_position + msg->target_position;
    motor.movement_speed = msg->movement_speed;
    RCLCPP_INFO(this->get_logger(), "Motor %d: Received new target position: %.2f degrees, target speed: %.2f",
                motor.motor_id, motor.target_position, motor.movement_speed);
}

void MotorController::updateMotorPosition() {
    // Motor Calibration is active, do not update motor position
    if (is_calibrating_) {
        return;
    }

    for (auto& motor : motors_) {
        if (!motor.calibrated) {
            RCLCPP_DEBUG(this->get_logger(), "Motor %d calibration required. Aborting motor control.", motor.motor_id);
            sendRecvMotorCmd(motor, queryMotorMode(motor.selected_motor_type, MotorMode::FOC), 0, 0, 0, 0, 0);
            continue;
        }

        // Calculate the distance to the target
        float distance_to_target = motor.target_position - motor.current_position;

        // Adjust the speed based on the distance to the target
        float speed = motor.movement_speed;
        if (fabs(distance_to_target) < motor.ramp_distance_) {
            speed = motor.movement_speed * (fabs(distance_to_target) / motor.ramp_distance_);
            if (speed < motor.min_speed_) speed = motor.min_speed_;
        }

        // Calculate the time interval in seconds from the publish rate
        float time_interval = 1.0 / motor_command_publish_rate_;

        // Calculate the incremental step based on the desired speed
        float increment = (speed * time_interval) * (distance_to_target > 0 ? 1 : -1);

        // Calculate the new position based on the speed
        float new_position = motor.current_position + increment;

        // Ensure the new position does not exceed the limits
        if (new_position < motor.min_position) {
            new_position = motor.min_position;
        } else if (new_position > motor.max_position) {
            new_position = motor.max_position;
        }

        // Convert the current position to rotor angle
        float rotor_angle_d = (new_position * (M_PI / 180)) * motor.gear_ratio;

        // Send the command to the motor
        sendRecvMotorCmd(motor, queryMotorMode(motor.selected_motor_type, MotorMode::FOC), 0, 0, rotor_angle_d, motor.rotor_kp, motor.rotor_kd);

        RCLCPP_DEBUG(this->get_logger(), "Motor %d: Current position: %.2f degrees, current speed: %.2f, movement speed: %.2f",
                     motor.motor_id, motor.current_position, motor.movement_speed, motor.current_velocity);
    }
}

void MotorController::publishJointState() {
    for (auto& motor : motors_) {
        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();
        joint_state_msg.name.push_back(motor.joint_name);

        // Calculate current angle relative to the initial position
        joint_state_msg.position.push_back((motor.current_position - motor.initial_position) * (M_PI / 180));
        joint_state_msg.velocity.push_back(motor.current_velocity);
        joint_state_msg.effort.push_back(motor.current_effort);
        motor.joint_state_pub_->publish(joint_state_msg);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}

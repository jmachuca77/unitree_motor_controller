#include "unitree_motor_controller/motor_controller.hpp"

MotorController::MotorController()
: Node("motor_controller"), serial_("/dev/ttyUSB0") {
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

    // set selected_motor_type_ based on the motor type
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
    //gear_ratio_ = queryGearRatio(MotorType::A1);
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

    // Set motor_cmd_.motorType and motor_data_.motorType based on the motor type
    motor_cmd_.motorType = selected_motor_type_;
    motor_data_.motorType = selected_motor_type_;

    motor_cmd_.mode = queryMotorMode(selected_motor_type_, MotorMode::BRAKE);
    motor_cmd_.id = motor_id_;
    motor_cmd_.kp = 0.0;
    motor_cmd_.kd = 0.0;
    motor_cmd_.q = 0.0;
    motor_cmd_.dq = 0.0;
    motor_cmd_.tau = 0.0;
    serial_.sendRecv(&motor_cmd_, &motor_data_);

    // Record the initial motor position
    initial_position_ = motor_data_.q / gear_ratio_ * (180 / M_PI);
    current_position_ = initial_position_;
    RCLCPP_INFO(this->get_logger(), "Initial motor position set to %.2f degrees as the reference.", initial_position_);

    // Initialize target position to initial position
    target_position_ = initial_position_;
}

MotorController::~MotorController() {
    // Set motor to BRAKE mode before shutting down
    RCLCPP_INFO(this->get_logger(), "Setting motor to BRAKE mode before shutdown.");
    // motor_cmd_.mode = queryMotorMode(MotorType::A1, MotorMode::BRAKE);
    motor_cmd_.mode = queryMotorMode(selected_motor_type_, MotorMode::BRAKE);
    serial_.sendRecv(&motor_cmd_, &motor_data_);
}

void MotorController::jointPositionCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Update the target position relative to the initial position
    target_position_ = initial_position_ + msg->data;
}

void MotorController::updateMotorPosition() {
    // Determine the distance to the target
    float distance_to_target = target_position_ - current_position_;

    // Adjust the speed based on the distance to the target
    float speed = max_speed_;
    if (fabs(distance_to_target) < ramp_distance_) {
        // Ramp-down speed
        speed = max_speed_ * (fabs(distance_to_target) / ramp_distance_);
        if (speed < min_speed_) speed = min_speed_;
    }

    // Calculate the time interval in seconds from the publish rate
    float time_interval = 1.0 / motor_command_publish_rate_;

    // Calculate the incremental step based on the desired speed
    float increment = (speed * time_interval) * (distance_to_target > 0 ? 1 : -1);

    // Update current position
    current_position_ += increment;

    // Convert the current position to rotor angle
    float rotor_angle_d = (current_position_ * (M_PI / 180)) * gear_ratio_;

    // Send the command to the motor
    motor_cmd_.motorType = selected_motor_type_;
    motor_data_.motorType = selected_motor_type_;
    motor_cmd_.mode  = queryMotorMode(selected_motor_type_, MotorMode::FOC);
    motor_cmd_.id    = motor_id_;
    motor_cmd_.kp    = rotor_kp_;
    motor_cmd_.kd    = rotor_kd_;
    motor_cmd_.q     = rotor_angle_d;
    motor_cmd_.dq    = 0.0;
    motor_cmd_.tau   = 0.0;
    serial_.sendRecv(&motor_cmd_, &motor_data_);

    float relative_position = current_position_ - initial_position_;
    RCLCPP_DEBUG(this->get_logger(), "Current position: %.2f degrees, Relative position: %.2f degrees", current_position_, relative_position);
}

void MotorController::publishJointState() {
    // Create and publish joint state message
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.push_back(joint_name_);
    
    // Calculate current angle relative to the initial position
    double current_position = motor_data_.q / gear_ratio_ * (180 / M_PI);
    joint_state_msg.position.push_back(current_position - initial_position_);
    joint_state_msg.velocity.push_back(motor_data_.dq / gear_ratio_ * (180 / M_PI));
    joint_state_msg.effort.push_back(motor_data_.tau);
    joint_state_msg.header.frame_id = joint_name_;
 
    joint_state_pub_->publish(joint_state_msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}

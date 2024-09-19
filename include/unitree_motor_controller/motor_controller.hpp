#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

class MotorController : public rclcpp::Node {
public:
    MotorController();
    ~MotorController(); // Destructor to handle cleanup

private:
    void jointPositionCallback(const std_msgs::msg::Float64::SharedPtr msg);
    void publishJointState();
    void updateMotorPosition();

    // Parameter change callback
    rcl_interfaces::msg::SetParametersResult parameterCallback(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr joint_position_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr motor_command_timer_;
    
    SerialPort serial_;
    MotorCmd motor_cmd_;
    MotorData motor_data_;

    float output_kp_ = 25;
    float output_kd_ = 0.6;
    float rotor_kp_ = 0;
    float rotor_kd_ = 0;

    float gear_ratio_;
    float initial_position_; // Initial motor position for relative measurements
    float current_position_; // Current motor position
    double target_position_; // Desired target position

    MotorType selected_motor_type_; // Motor type

    // ROS Parameters
    std::string motor_type_;  // Parameter to set the motor type
    int motor_id_; // Parameter to set the motor ID
    std::string joint_name_; // Parameter to set the joint name
    int joint_state_publish_rate_;
    int motor_command_publish_rate_;

    float max_position_; // Maximum position limit
    float min_position_; // Minimum position limit
    float max_speed_ = 300; // Maximum speed for motor movement
    float min_speed_ = 5.0;  // Minimum speed limit to avoid stopping too early
    float ramp_distance_ = 10.0; // Distance within which to start slowing down
};

#endif // MOTOR_CONTROLLER_HPP

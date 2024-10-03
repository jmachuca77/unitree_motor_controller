#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <yaml-cpp/yaml.h>
#include "bdx_msgs/msg/joint_position_target.hpp"
#include "std_srvs/srv/set_bool.hpp"

class MotorController : public rclcpp::Node {
public:
    MotorController();
    ~MotorController(); // Destructor to handle cleanup

private:
    // Calibration service related functions
    void startCalibration(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void stopCalibration(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void calibrationLoop();

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_calibration_service_;
    rclcpp::TimerBase::SharedPtr calibration_timer_;

    // Motor control related functions
    void sendRecvMotorCmd(unsigned short id, unsigned short mode, float tau, float dq, float q, float kp, float kd);
    void jointPositionCallback(const bdx_msgs::msg::JointPositionTarget::SharedPtr msg);
    void publishJointState();
    void updateMotorPosition();
    
    // New functions for motor limits management and calibration
    bool loadMotorLimits(); // Load motor limits from YAML file
    bool performMotorCalibration(); // Perform calibration if limits are not available
    bool saveMotorLimits(); // Save motor limits to YAML file

    // ROS Parameters
    rclcpp::Subscription<bdx_msgs::msg::JointPositionTarget>::SharedPtr joint_position_sub_;
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
    float current_velocity_; // Current motor velocity
    float current_effort_; // Current motor effort
    float movement_speed_; // Speed of motor movement

    MotorType selected_motor_type_; // Motor type

    // New members for motor limits
    bool calibrated_ = false;
    bool is_calibrating_ = false;
    float max_position_ = std::numeric_limits<float>::lowest(); // Maximum position limit
    float min_position_ = std::numeric_limits<float>::max(); // Minimum position limit
    float max_speed_ = 300; // Maximum speed for motor movement
    float min_speed_ = 5.0;  // Minimum speed limit to avoid stopping too early
    float ramp_distance_ = 10.0; // Distance within which to start slowing down

    // ROS Parameters
    std::string motor_type_;  // Parameter to set the motor type
    int motor_id_; // Parameter to set the motor ID
    std::string joint_name_; // Parameter to set the joint name
    int joint_state_publish_rate_;
    int motor_command_publish_rate_;

    std::string config_file_; // Parameter to set the config file path
};

#endif // MOTOR_CONTROLLER_HPP

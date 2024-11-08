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
#include <vector>
#include <map>

class MotorController : public rclcpp::Node {
public:
    MotorController();
    ~MotorController(); // Destructor to handle cleanup

private:
    // Motor parameters for each motor
    struct MotorParameters {
        int motor_id;
        std::string joint_name;
        std::string motor_type;
        MotorType selected_motor_type;

        float output_kp;
        float output_kd;
        float rotor_kp;
        float rotor_kd;
        float gear_ratio;

        float initial_position; // Initial motor position for relative measurements
        float current_position; // Current motor position
        double target_position; // Desired target position
        float current_velocity; // Current motor velocity
        float current_effort; // Current motor effort
        float movement_speed; // Speed of motor movement

        bool calibrated;
        bool is_calibrating;
        float max_position; // Maximum position limit
        float min_position; // Minimum position limit

        float max_speed_; // Maximum speed for motor movement
        float min_speed_;  // Minimum speed limit to avoid stopping too early
        float ramp_distance_; // Distance within which to start slowing down
        
        MotorCmd motor_cmd;
        MotorData motor_data;

        // ROS Publishers and Subscribers
        rclcpp::Subscription<bdx_msgs::msg::JointPositionTarget>::SharedPtr joint_position_sub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    };

    // Calibration service related functions
    void startCalibration(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                          std::shared_ptr<std_srvs::srv::SetBool::Response> response);
    void calibrationLoop();

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_calibration_service_;
    rclcpp::TimerBase::SharedPtr calibration_timer_;

    // Motor control related functions
    void sendRecvMotorCmd(MotorParameters& motor, unsigned short mode, float tau, float dq, float q, float kp, float kd);
    void jointPositionCallback(const bdx_msgs::msg::JointPositionTarget::SharedPtr msg, int motor_index);
    void publishJointState();
    void updateMotorPosition();

    bool loadMotorLimits(); // Load motor limits from YAML file
    bool saveMotorLimits(); // Save motor limits to YAML file

    // ROS Parameters
    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr motor_command_timer_;

    SerialPort serial_;

    // New members for motor limits
    bool calibrated_;
    bool is_calibrating_;


    // ROS Parameters
    std::vector<int> motor_ids_; // Array of motor IDs
    std::string config_file_; // Parameter to set the config file path
    int joint_state_publish_rate_;
    int motor_command_publish_rate_;

    // Collection of motors
    std::vector<MotorParameters> motors_;
};

#endif // MOTOR_CONTROLLER_HPP

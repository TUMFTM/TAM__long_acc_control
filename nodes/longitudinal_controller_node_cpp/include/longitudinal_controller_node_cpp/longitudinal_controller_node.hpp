// Copyright 2023 Simon Sagmeister
#pragma once

#include <algorithm>
#include <memory>
// ROS
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/rclcpp.hpp>

// messages
#include <nav_msgs/msg/odometry.hpp>

#include "autoware_auto_control_msgs/msg/ackermann_control_command.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tum_msgs/msg/tum_control_constraint_point.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_msgs/msg/tum_int8_stamped.hpp"
#include "tum_msgs/msg/tum_bool_stamped.hpp"
//
#include "tum_msgs/msg/tum_longitudinal_cmd.hpp"

// longitudinal controller class
#include "long_acc_controller_tam_cpp/long_acc_controller.hpp"

// Parameter handling
#include <param_management_ros2_integration_cpp/helper_functions.hpp>

// Debug Handling
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"

// type conversions
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
// Timeouting and diagnostic publishing
#include "longitudinal_controller_state_machine.hpp"
#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/timeout_value_provider.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"
// Include the tum helpers for the function queue
#include "tum_helpers_cpp/containers.hpp"

namespace tam::control
{
class LongitudinalControllerNode : public rclcpp::Node
{
public:
  explicit LongitudinalControllerNode(
    std::unique_ptr<tam::control::LongAccControllerCpp> && longitudinal_controller,
    const rclcpp::NodeOptions & options);

private:
  std::unique_ptr<tam::control::LongAccControllerCpp> longitudinal_controller_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // Callback queue objects
  rclcpp::TimerBase::SharedPtr model_update_timer_;

  // Node monitor
  tam::core::NodeMonitor::UniquePtr node_monitor_;
  std::unique_ptr<LongitudinalControllerStateMachine> longitudinal_controller_state_machine_;

  // Topic Watchdog
  tam::core::TopicWatchdog::UniquePtr topic_watchdog_;

  tam::types::control::DriveTrainFeedback drivetrain;

  // Timer callbacks
  void model_update_callback();
  void evaluate_diagnostics();
  void function_queue_callback();

  // Subscription callbacks
  void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);
  void omega_engine_callback(const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg);
  void gear_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg);
  void brake_pressure_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  void wheelspeeds_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void steering_report_callback(
    const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg);
  void target_acceleration_callback(
    const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg);
  void brake_warmup_pressure_callback(
    const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  void gear_request_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg);
  void slip_control_active_callback(const tum_msgs::msg::TUMBoolStamped::SharedPtr msg);

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    sub_acceleration_{};
  rclcpp::Subscription<tum_msgs::msg::TUMFloat32Stamped>::SharedPtr sub_omega_engine_{};
  rclcpp::Subscription<tum_msgs::msg::TUMInt8Stamped>::SharedPtr sub_gear_{};
  rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_brake_pressure_{};
  rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr sub_wheelspeed_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_{};
  rclcpp::Subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>::SharedPtr
    sub_steering_report_{};
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr
    sub_long_acc_target_{};
  rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr
    sub_warmup_brake_pressure_{};
  rclcpp::Subscription<tum_msgs::msg::TUMInt8Stamped>::SharedPtr sub_gear_request_{};
  rclcpp::Subscription<tum_msgs::msg::TUMBoolStamped>::SharedPtr sub_slip_control_active_{};

  // Publishers
  rclcpp::Publisher<tum_msgs::msg::TUMLongitudinalCmd>::SharedPtr ctrl_cmd_pub_{};

  // Debugging
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  tam::tsl::LoggerComposer::SharedPtr logger_composer_;
  tam::tsl::TSLPublisher tsl_publisher_;

  // Timing
  std::chrono::steady_clock::time_point callback_start_time;
  std::chrono::steady_clock::time_point last_call_time;
  u_int32_t cycle_count_;
};
}  // namespace tam::control

// Copyright 2024 Sven Goblirsch
#pragma once

#include <cmath>

#include <chrono>
#include <memory>
#include <param_management_ros2_integration_cpp/helper_functions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "gear_shift_controller_tam_cpp/gear_shift_controller.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/timeout_value_provider.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"
#include "tier4_planning_msgs/msg/trajectory.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_msgs/msg/tum_additional_info_point.hpp"
#include "tum_msgs/msg/tum_control_constraints.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_msgs/msg/tum_int8_stamped.hpp"
#include "tum_msgs/msg/tum_longitudinal_cmd.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
class GearShiftControllerNode : public rclcpp::Node
{
public:
  explicit GearShiftControllerNode(
    std::unique_ptr<tam::control::GearShiftController> && gear_shift_controller,
    const rclcpp::NodeOptions & options);

private:
  // Controller
  std::unique_ptr<tam::control::GearShiftController> gear_shift_controller_;

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  // node monitor
  tam::core::NodeMonitor::UniquePtr monitor_ = std::make_unique<tam::core::NodeMonitor>(this);
  // Callback queue
  rclcpp::TimerBase::SharedPtr model_update_timer_{};

  // Subscriptions
  message_filters::Subscriber<tier4_planning_msgs::msg::Trajectory> sub_trajectory_;
  message_filters::Subscriber<tum_msgs::msg::TUMControlConstraints> sub_constraints_;
  std::shared_ptr<message_filters::TimeSynchronizer<
    tier4_planning_msgs::msg::Trajectory, tum_msgs::msg::TUMControlConstraints>>
    sync_;
  rclcpp::Subscription<tum_msgs::msg::TUMFloat32Stamped>::SharedPtr omega_eng_subs_{};
  rclcpp::Subscription<tum_msgs::msg::TUMInt8Stamped>::SharedPtr current_gear_subs_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subs_{};
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr
    acceleration_subs_{};

  // Publisher
  rclcpp::Publisher<tum_msgs::msg::TUMInt8Stamped>::SharedPtr gear_request_pub_{};
  tam::tsl::TSLPublisher tsl_publisher_;

  // Flags
  bool odom_received_{false};
  bool traj_received_{false};

  //  Subscription callbacks
  void omega_engine_callback(const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg);
  void gear_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg);
  void trajectory_callback(
    const tier4_planning_msgs::msg::Trajectory::ConstSharedPtr & traj,
    const tum_msgs::msg::TUMControlConstraints::ConstSharedPtr & constr);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg);

  // Node Functions
  void gear_shift_controller_callback();
};
}  // namespace tam::control

// Copyright 2023 Tobias Betz
#pragma once

#include <rclcpp/rclcpp.hpp>

#include "controller_helpers_cpp/helpers.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "ros2_watchdog_cpp/timeout_value_provider.hpp"
#include "ros2_watchdog_cpp/topic_watchdog.hpp"
#include "tier4_planning_msgs/msg/trajectory.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
#include "tum_msgs/msg/tum_additional_info_point.hpp"
#include "tum_msgs/msg/tum_additional_trajectory_infos.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_msgs/msg/tum_longitudinal_cmd.hpp"
#include "tum_types_cpp/common.hpp"
namespace tam::control
{
class BrakeTemperatureControllerNode : public rclcpp::Node
{
public:
  explicit BrakeTemperatureControllerNode(const rclcpp::NodeOptions & options);

private:
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  tam::tsl::TSLPublisher tsl_publisher_{this, logger_};
  tam::core::NodeMonitor::UniquePtr monitor_ = std::make_unique<tam::core::NodeMonitor>(this);

  // Member Variables
  bool lap_condition_flag_ = true;
  bool velocity_condition_flag_ = true;
  bool temperature_threshold_flag_ = true;
  bool temperature_threshold_flag_front_ = true;
  bool temperature_threshold_flag_rear_ = true;
  bool temperature_inplausibility_flag_ = true;
  bool received_brake_temperature_ = false;
  bool acceleration_ax_flag_ = false;
  bool acceleration_ay_flag_ = false;
  int lap_counter_ = 0;
  tum_msgs::msg::TUMAdditionalInfoPoint tum_additional_info_{};
  float brake_pressure_scale_ay_ = 0.0;
  float brake_pressure_scale_ax_ = 0.0;
  float brake_pressure_scale_temp_front_ = 0.0;
  float brake_pressure_scale_temp_rear_ = 0.0;
  float warm_up_brake_pressure_front_pa_ = 0.0;
  float warm_up_brake_pressure_rear_pa_ = 0.0;
  float ay_accel_gradient_;
  float ax_accel_gradient_;
  float temperature_gradient_;
  float ay_accel_ = 0.0;
  float ax_accel_ = 0.0;
  float vehicle_velocity_ = 0.0;
  float brake_temperature_fl_ = 0.0;
  float brake_temperature_fr_ = 0.0;
  float brake_temperature_rl_ = 0.0;
  float brake_temperature_rr_ = 0.0;
  float brake_temperature_front_axis_ = 0.0;
  float brake_temperature_rear_axis_ = 0.0;

  tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr warm_up_brake_pressure_cmd_ =
    std::make_shared<tum_msgs::msg::TUMFloat64PerWheelStamped>();

  tam::helpers::control::FirstOrderLowPass<double> temperature_filter_front_axis_{40.0, 0.95};
  tam::helpers::control::FirstOrderLowPass<double> temperature_filter_rear_axis_{40.0, 0.95};
  tam::helpers::control::FirstOrderLowPass<double> ax_filter_{0.0, 0.9};
  tam::helpers::control::FirstOrderLowPass<double> ay_filter_{0.0, 0.9};

  // params
  float max_warm_up_brake_pressure_pa;
  bool warm_up_disabled_flag;
  int max_lap_warm_up;
  float ay_x1;
  float ay_x2;
  float ax_x1;
  float ax_x2;
  float temperature_x1;
  float threshold_brake_temperature;

  // Subscriptions
  rclcpp::Subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr
    brake_temperature_subs_{};
  rclcpp::Subscription<tier4_planning_msgs::msg::Trajectory>::SharedPtr trajectory_subs_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subs_{};
  rclcpp::Subscription<tum_msgs::msg::TUMAdditionalTrajectoryInfos>::SharedPtr lap_info_subs_{};

  // Publisher
  rclcpp::Publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>::SharedPtr
    warm_up_brake_pressure_pub_{};

  //  Subscription callbacks
  void brake_temperature_callback(const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg);
  // void acceleration_callback(const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr
  // msg);
  void trajectory_acceleration_callback(const tier4_planning_msgs::msg::Trajectory::SharedPtr msg);
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void lap_info_callback(const tum_msgs::msg::TUMAdditionalTrajectoryInfos::SharedPtr msg);

  // Custom Functions
  void calculateBrakePressure();
  void getBrakePressure_ay_acceleration();
  void getBrakePressure_ax_acceleration();
  float getBrakePressure_temperature(float brake_temp);
  bool warm_up_disabled()
  {
    return (
      warm_up_disabled_flag || temperature_inplausibility_flag_ || temperature_threshold_flag_ ||
      lap_condition_flag_ || velocity_condition_flag_ || acceleration_ax_flag_ ||
      acceleration_ay_flag_);
  }
};
}  // namespace tam::control

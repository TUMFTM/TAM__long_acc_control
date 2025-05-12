// Copyright 2023 Tobias Betz
#include "brake_temperature_controller_node_cpp/brake_temperature_controller_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
namespace tam::control
{
BrakeTemperatureControllerNode::BrakeTemperatureControllerNode(const rclcpp::NodeOptions & options)
: Node("BrakeTemperatureControllerNode", "/core/control", options)
{
  this->declare_parameter<float>("max_warm_up_brake_pressure_pa", 5'00'000.0);
  this->get_parameter("max_warm_up_brake_pressure_pa", max_warm_up_brake_pressure_pa);
  this->declare_parameter<bool>("warm_up_disabled", true);
  this->get_parameter("warm_up_disabled", warm_up_disabled_flag);
  this->declare_parameter<int>("max_lap_warm_up", -1);  // -1 disables lap condition
  this->get_parameter("max_lap_warm_up", max_lap_warm_up);
  this->declare_parameter<float>("threshold_brake_temperature", 470.0);
  this->get_parameter("threshold_brake_temperature", threshold_brake_temperature);
  this->declare_parameter<float>("ay_x1", 2.0);
  this->get_parameter("ay_x1", ay_x1);
  this->declare_parameter<float>("ay_x2", 5.0);
  this->get_parameter("ay_x2", ay_x2);
  this->declare_parameter<float>("ax_x1", -3.5);
  this->get_parameter("ax_x1", ax_x1);
  this->declare_parameter<float>("ax_x2", -2.0);
  this->get_parameter("ax_x2", ax_x2);

  this->declare_parameter<float>("temperature_x1", 400.0);
  this->get_parameter("temperature_x1", temperature_x1);

  // Todo: Data Type to make the code nicer
  // Todo: ROS 2 Params to deactivate warm up

  // Calculate coefficient for linear equation
  if (
    (ay_x2 - ay_x1) < 0.01 || (threshold_brake_temperature - temperature_x1) < 0.01 ||
    (ax_x2 - ax_x1) < 0.01) {
    RCLCPP_INFO(
      this->get_logger(), "Error: Wrong parameters choosen resulting in too large gradients");
    rclcpp::shutdown();
    return;
  }
  ay_accel_gradient_ = -1 / (ay_x2 - ay_x1);
  temperature_gradient_ = -1 / (threshold_brake_temperature - temperature_x1);
  ax_accel_gradient_ = 1 / (ax_x2 - ax_x1);

  // Subscriptions
  brake_temperature_subs_ = this->create_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    "/vehicle/sensor/brake_temperature_degree", 1,
    std::bind(&BrakeTemperatureControllerNode::brake_temperature_callback, this, _1));

  trajectory_subs_ = this->create_subscription<tier4_planning_msgs::msg::Trajectory>(
    "/core/planning/target_trajectory/trajectory", 1,
    std::bind(&BrakeTemperatureControllerNode::trajectory_acceleration_callback, this, _1));

  odometry_subs_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/core/state/odometry", 1,
    std::bind(&BrakeTemperatureControllerNode::odometry_callback, this, _1));

  lap_info_subs_ = this->create_subscription<tum_msgs::msg::TUMAdditionalTrajectoryInfos>(
    "/core/planning/target_trajectory/additional_info", 1,
    std::bind(&BrakeTemperatureControllerNode::lap_info_callback, this, _1));

  // Publisher
  warm_up_brake_pressure_pub_ = this->create_publisher<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    "/core/control/warm_up_brake_pressure_pa", 1);
}
void BrakeTemperatureControllerNode::calculateBrakePressure()
{
  // Calculate brake warmup pressure
  this->getBrakePressure_ay_acceleration();
  this->getBrakePressure_ax_acceleration();

  // Calculate brake Pressure scaling for front axis
  brake_pressure_scale_temp_front_ =
    this->getBrakePressure_temperature(brake_temperature_front_axis_);
  warm_up_brake_pressure_front_pa_ = brake_pressure_scale_ay_ * brake_pressure_scale_ax_ *
                                     brake_pressure_scale_temp_front_ *
                                     max_warm_up_brake_pressure_pa;

  // Calculate brake pressure scaling for rear axis
  brake_pressure_scale_temp_rear_ =
    this->getBrakePressure_temperature(brake_temperature_rear_axis_);
  warm_up_brake_pressure_rear_pa_ = brake_pressure_scale_ay_ * brake_pressure_scale_ax_ *
                                    brake_pressure_scale_temp_rear_ * max_warm_up_brake_pressure_pa;

  warm_up_disabled_flag = this->get_parameter("warm_up_disabled").as_bool();

  if (warm_up_disabled()) {
    builtin_interfaces::msg::Time stamp = get_clock()->now();
    warm_up_brake_pressure_cmd_->stamp = stamp;
    warm_up_brake_pressure_cmd_->data.front_left = 0.0;
    warm_up_brake_pressure_cmd_->data.front_right = 0.0;
    warm_up_brake_pressure_cmd_->data.rear_left = 0.0;
    warm_up_brake_pressure_cmd_->data.rear_right = 0.0;
    warm_up_brake_pressure_pub_->publish(*warm_up_brake_pressure_cmd_);

    monitor_->set_error_lvl("Warmup Mode", tam::types::ErrorLvl::OK);
    monitor_->set_message("No Brake Disc Warm Up");
    // Add Sägis Magic Type
    monitor_->report_value(
      "max_temp", std::max(
                    {brake_temperature_fl_, brake_temperature_fr_, brake_temperature_rl_,
                     brake_temperature_rr_}));
    monitor_->report_value(
      "min_temp", std::min(
                    {brake_temperature_fl_, brake_temperature_fr_, brake_temperature_rl_,
                     brake_temperature_rr_}));
  } else {
    warm_up_brake_pressure_cmd_->data.front_left = warm_up_brake_pressure_front_pa_;
    warm_up_brake_pressure_cmd_->data.front_right = warm_up_brake_pressure_front_pa_;
    warm_up_brake_pressure_cmd_->data.rear_left = warm_up_brake_pressure_rear_pa_;
    warm_up_brake_pressure_cmd_->data.rear_right = warm_up_brake_pressure_rear_pa_;

    // Cases for temperature per axis > threshold, overwrite with pressure from controller
    if (temperature_threshold_flag_front_) {
      warm_up_brake_pressure_cmd_->data.front_left = 0.0;
      warm_up_brake_pressure_cmd_->data.front_right = 0.0;
    } else if (temperature_threshold_flag_rear_) {
      warm_up_brake_pressure_cmd_->data.rear_left = 0.0;
      warm_up_brake_pressure_cmd_->data.rear_right = 0.0;
    }
    builtin_interfaces::msg::Time stamp = get_clock()->now();
    warm_up_brake_pressure_cmd_->stamp = stamp;
    warm_up_brake_pressure_pub_->publish(*warm_up_brake_pressure_cmd_);

    monitor_->set_error_lvl("Warmup Mode", tam::types::ErrorLvl::WARN);
    monitor_->set_message("Brake Disc Warm Up");
    monitor_->report_value(
      "max_temp", std::max(
                    {brake_temperature_fl_, brake_temperature_fr_, brake_temperature_rl_,
                     brake_temperature_rr_}));
    monitor_->report_value(
      "min_temp", std::min(
                    {brake_temperature_fl_, brake_temperature_fr_, brake_temperature_rl_,
                     brake_temperature_rr_}));
  }
  logger_->log("lap_condition_flag", lap_condition_flag_);
  logger_->log("warm_up_disabled_flag", warm_up_disabled_flag);
  logger_->log("velocity_condition_flag", velocity_condition_flag_);
  logger_->log("temperature_threshold_flag", temperature_threshold_flag_);
  logger_->log("temperature_threshold_flag_front", temperature_threshold_flag_front_);
  logger_->log("temperature_threshold_flag_rear", temperature_threshold_flag_rear_);
  logger_->log("temperature_inplausibility_flag", temperature_inplausibility_flag_);
  logger_->log("lap_counter", lap_counter_);
  logger_->log("brake_pressure_scale_ay", brake_pressure_scale_ay_);
  logger_->log("brake_pressure_scale_ax", brake_pressure_scale_ax_);
  logger_->log("brake_pressure_scale_temp_rear", brake_pressure_scale_temp_rear_);
  logger_->log("brake_pressure_scale_temp_front", brake_pressure_scale_temp_front_);
  logger_->log("warm_up_brake_pressure_front_pa", warm_up_brake_pressure_front_pa_);
  logger_->log("warm_up_brake_pressure_rear_pa", warm_up_brake_pressure_rear_pa_);
  logger_->log("ay_accel_gradient", ay_accel_gradient_);
  logger_->log("ax_accel_gradient", ax_accel_gradient_);
  logger_->log("temperature_gradient", temperature_gradient_);
  logger_->log("ay_accel", ay_accel_);
  logger_->log("ax_accel", ax_accel_);
  logger_->log("acceleration_ax_flag", acceleration_ax_flag_);
  logger_->log("acceleration_ay_flag", acceleration_ay_flag_);
  logger_->log("brake_temperature_fl", brake_temperature_fl_);
  logger_->log("brake_temperature_fr", brake_temperature_fr_);
  logger_->log("brake_temperature_rl", brake_temperature_rl_);
  logger_->log("brake_temperature_rr", brake_temperature_rr_);
  logger_->log("brake_temperature_front_axis", brake_temperature_front_axis_);
  logger_->log("brake_temperature_rear_axis", brake_temperature_rear_axis_);

  tsl_publisher_.trigger();
}
void BrakeTemperatureControllerNode::brake_temperature_callback(
  const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  if (!received_brake_temperature_) {
    // Initiliazation finished after first subscription
    monitor_->initialization_finished();
    received_brake_temperature_ = true;
  }

  brake_temperature_fl_ = msg->data.front_left;
  brake_temperature_fr_ = msg->data.front_right;
  brake_temperature_rl_ = msg->data.rear_left;
  brake_temperature_rr_ = msg->data.rear_right;

  // Add plausibility check
  if (
    std::min(
      {brake_temperature_fl_, brake_temperature_fr_, brake_temperature_rl_,
       brake_temperature_rr_}) < 5.0) {
    temperature_inplausibility_flag_ = true;
    return;
  } else {
    temperature_inplausibility_flag_ = false;
  }

  brake_temperature_front_axis_ =
    temperature_filter_front_axis_.step((brake_temperature_fl_ + brake_temperature_fr_) / 2.0);
  brake_temperature_rear_axis_ =
    temperature_filter_rear_axis_.step((brake_temperature_rl_ + brake_temperature_rr_) / 2.0);

  // If individual axis are above threshold set axis flag to disable warm up mode for this
  if (brake_temperature_front_axis_ > threshold_brake_temperature) {
    temperature_threshold_flag_front_ = true;
  } else {
    temperature_threshold_flag_front_ = false;
  }

  if (brake_temperature_rear_axis_ > threshold_brake_temperature) {
    temperature_threshold_flag_rear_ = true;
  } else {
    temperature_threshold_flag_rear_ = false;
  }

  // If all break discs are above threshold set global flag to disable warm up mode
  if (
    std::min({brake_temperature_front_axis_, brake_temperature_rear_axis_}) >
    threshold_brake_temperature) {
    temperature_threshold_flag_ = true;
  } else {
    temperature_threshold_flag_ = false;
  }

  this->calculateBrakePressure();
  monitor_->update();
}
void BrakeTemperatureControllerNode::trajectory_acceleration_callback(
  const tier4_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  ax_accel_ = msg->points[0].accel.linear.x;
  ax_accel_ = ax_filter_.step(ax_accel_);
  ay_accel_ = std::abs(msg->points[0].accel.linear.y);
  ay_accel_ = ay_filter_.step(ay_accel_);

  if (ax_accel_ < ax_x1) {
    acceleration_ax_flag_ = true;
  } else {
    acceleration_ax_flag_ = false;
  }

  if (ay_accel_ > ay_x2) {
    acceleration_ay_flag_ = true;
  } else {
    acceleration_ay_flag_ = false;
  }
}
void BrakeTemperatureControllerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (msg->twist.twist.linear.x < 13.0) {
    velocity_condition_flag_ = true;
  } else if (msg->twist.twist.linear.x > 18.0) {
    velocity_condition_flag_ = false;
  }

  return;
}
void BrakeTemperatureControllerNode::lap_info_callback(
  const tum_msgs::msg::TUMAdditionalTrajectoryInfos::SharedPtr msg)
{

  tum_additional_info_ = msg->points[3];
  lap_counter_ = int(tum_additional_info_.lap_cnt);
  
  if (max_lap_warm_up == -1) {
    lap_condition_flag_ = false;
    return;
  }


  if (lap_counter_ < max_lap_warm_up) {
    lap_condition_flag_ = false;
  } else {
    lap_condition_flag_ = true;
  }
}
// Todo: Write single function to handle all different scalings
void BrakeTemperatureControllerNode::getBrakePressure_ay_acceleration()
{
  if (ay_accel_ < ay_x1) {
    brake_pressure_scale_ay_ = 1.0;
  } else if (ay_accel_ >= ay_x1 && ay_accel_ < ay_x2) {
    brake_pressure_scale_ay_ = ay_accel_gradient_ * (ay_accel_ - ay_x1) + 1.0;
  } else {
    brake_pressure_scale_ay_ = 0.0;
  }
}
void BrakeTemperatureControllerNode::getBrakePressure_ax_acceleration()
{
  if (ax_accel_ < ax_x1) {
    brake_pressure_scale_ax_ = 0.0;  // Case for negative acceleration
  } else if (ax_accel_ >= ax_x1 && ax_accel_ < ax_x2) {
    brake_pressure_scale_ax_ = ax_accel_gradient_ * (ax_accel_ - ax_x1);
  } else {
    brake_pressure_scale_ax_ = 1.0;
  }
}
float BrakeTemperatureControllerNode::getBrakePressure_temperature(float brake_temp)
{
  float brake_pressure_scale_temp = 0.0;
  if (brake_temp < temperature_x1) {
    brake_pressure_scale_temp = 1.0;
  } else if (brake_temp >= temperature_x1 && brake_temp < threshold_brake_temperature) {
    brake_pressure_scale_temp = temperature_gradient_ * (brake_temp - temperature_x1) + 1.0;
  } else {
    brake_pressure_scale_temp = 0.0;
  }
  return brake_pressure_scale_temp;
}
}  // namespace tam::control

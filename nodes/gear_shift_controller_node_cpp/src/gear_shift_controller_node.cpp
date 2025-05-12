// Copyright 2024 Sven Goblirsch
#include "gear_shift_controller_node_cpp/gear_shift_controller_node.hpp"

#include "tum_ros_helpers_cpp/timer.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
namespace tam::control
{
GearShiftControllerNode::GearShiftControllerNode(
  std::unique_ptr<tam::control::GearShiftController> && gear_shift_controller,
  const rclcpp::NodeOptions & options)
: Node("GearShiftControllerNode", "/core/control", options),
  gear_shift_controller_(std::move(gear_shift_controller)),
  tsl_publisher_(this, gear_shift_controller_->get_debug_out())
{
  callback_handle_ =
    tam::pmg::connect_param_manager_to_ros_cb(this, gear_shift_controller_->get_param_manager());
  // Declare all paramters from the param manager
  tam::pmg::declare_ros_params_from_param_manager(
    this, gear_shift_controller_->get_param_manager().get());

  //  Check Parameter
  for (std::string name : this->list_parameters({}, 1).names) {
    rclcpp::Parameter param = this->get_parameter(name);
    RCLCPP_DEBUG(
      this->get_logger(), "param name: %s, value: %s", param.get_name().c_str(),
      param.value_to_string().c_str());
  }

  RCLCPP_DEBUG(this->get_logger(), "Parameter Manager of Gear Shift Controller initialized");
  using namespace std::chrono_literals;
  {
    model_update_timer_ = tam::create_timer(
      this, 20ms, std::bind(&GearShiftControllerNode::gear_shift_controller_callback, this));
  }
  // Subscriptions
  omega_eng_subs_ = this->create_subscription<tum_msgs::msg::TUMFloat32Stamped>(
    "/vehicle/sensor/omega_engine_radps", 1,
    std::bind(&GearShiftControllerNode::omega_engine_callback, this, _1));
  current_gear_subs_ = this->create_subscription<tum_msgs::msg::TUMInt8Stamped>(
    "/vehicle/sensor/gear", 1, std::bind(&GearShiftControllerNode::gear_callback, this, _1));
  odometry_subs_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/core/state/odometry", 1, std::bind(&GearShiftControllerNode::odometry_callback, this, _1));
  acceleration_subs_ = this->create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "/core/state/acceleration", 1,
    std::bind(&GearShiftControllerNode::acceleration_callback, this, _1));

  sync_ = std::make_shared<message_filters::TimeSynchronizer<
    tier4_planning_msgs::msg::Trajectory, tum_msgs::msg::TUMControlConstraints>>(
    sub_trajectory_, sub_constraints_, 1);
  sync_->registerCallback(std::bind(&GearShiftControllerNode::trajectory_callback, this, _1, _2));

  sub_trajectory_.subscribe(this, "/core/planning/target_trajectory/trajectory");
  sub_constraints_.subscribe(this, "/core/planning/target_trajectory/constraints");
  // Publisher
  gear_request_pub_ =
    this->create_publisher<tum_msgs::msg::TUMInt8Stamped>("/core/control/gear_request", 1);

  monitor_->initialization_finished();
  monitor_->set_error_lvl("Initializing", tam::types::ErrorLvl::WARN);
  monitor_->set_message("Initializing");
  monitor_->report_value("upshift_flag", gear_shift_controller_->get_upshift_flag());
  monitor_->report_value("downshift_flag", gear_shift_controller_->get_downshift_flag());
  monitor_->update();
}
void GearShiftControllerNode::gear_shift_controller_callback()
{
  if (!odom_received_ || !traj_received_) {
    monitor_->set_error_lvl("Initializing", tam::types::ErrorLvl::ERROR);
    monitor_->set_message("Waiting for odometry or trajectory!");
    monitor_->update();
    return;
  } else {
    monitor_->set_error_lvl("Initializing", tam::types::ErrorLvl::OK);
    monitor_->set_message("OK");
  }

  // step gear shift controller
  gear_shift_controller_->step();

  // publish gear request
  builtin_interfaces::msg::Time stamp = get_clock()->now();
  tum_msgs::msg::TUMInt8Stamped msg_gear_request;
  msg_gear_request.stamp = stamp;
  msg_gear_request.data = gear_shift_controller_->get_gear_command();
  gear_request_pub_->publish(msg_gear_request);

  // diagnostics
  monitor_->report_value("upshift_flag", gear_shift_controller_->get_upshift_flag());
  monitor_->report_value("downshift_flag", gear_shift_controller_->get_downshift_flag());
  monitor_->update();

  // debug
  tsl_publisher_.trigger();
}
void GearShiftControllerNode::omega_engine_callback(
  const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg)
{
  gear_shift_controller_->set_feedback_omega_engine(msg->data);
}
void GearShiftControllerNode::gear_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg)
{
  gear_shift_controller_->set_feedback_gear(msg->data);
}
void GearShiftControllerNode::trajectory_callback(
  const tier4_planning_msgs::msg::Trajectory::ConstSharedPtr & traj,
  const tum_msgs::msg::TUMControlConstraints::ConstSharedPtr & constr)
{
  traj_received_ = true;
  gear_shift_controller_->set_feedback_trajectory(
    tam::type::conversions::cpp::Trajectory_type_from_msg(traj),
    tam::type_conversions::constraint_type_from_msg(*constr));
}
void GearShiftControllerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_received_ = true;
  gear_shift_controller_->set_feedback_odometry(
    tam::type::conversions::cpp::Odometry_type_from_msg(msg));
}
void GearShiftControllerNode::acceleration_callback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  gear_shift_controller_->set_feedback_acceleration(msg->accel.accel.linear.y);
}
}  // namespace tam::control
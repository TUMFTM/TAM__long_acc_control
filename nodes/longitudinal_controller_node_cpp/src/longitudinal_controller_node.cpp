// Copyright 2023 Daniel Esser
#include "longitudinal_controller_node_cpp/longitudinal_controller_node.hpp"
#include "tum_ros_helpers_cpp/qos.hpp"
#include "tum_ros_helpers_cpp/timer.hpp"

using std::placeholders::_1;
namespace tam::control
{
LongitudinalControllerNode::LongitudinalControllerNode(
  std::unique_ptr<tam::control::LongAccControllerCpp> && longitudinal_controller,
  const rclcpp::NodeOptions & options)
: Node("LongitudinalController", "/core/control", options),
  longitudinal_controller_(std::move(longitudinal_controller)),
  logger_composer_(std::make_shared<tam::tsl::LoggerComposer>(
    std::vector<tam::tsl::LoggerAccessInterface::SharedPtr>(
      {logger_, longitudinal_controller_->get_debug_out()}))),
  tsl_publisher_(tam::tsl::TSLPublisher(this, logger_composer_))
{
  // Debug Timing
  cycle_count_ = 0;

  // Node monitor
  node_monitor_ = std::make_unique<tam::core::NodeMonitor>(this);
  // Topic Watchdog
  topic_watchdog_ = std::make_unique<tam::core::TopicWatchdog>(this);
  // State Machine
  longitudinal_controller_state_machine_ = std::make_unique<LongitudinalControllerStateMachine>();
  // Timeout Value Provider
  tam::core::TimeoutValueProvider timeout_values;

  // parameter handling
  callback_handle_ =
    tam::pmg::connect_param_manager_to_ros_cb(this, longitudinal_controller_->get_param_handler());
  // Declare all paramters from the param manager
  tam::pmg::declare_ros_params_from_param_manager(
    this, longitudinal_controller_->get_param_handler().get());
  if (!this->has_parameter("tS_LongitudinalController")) {
    this->declare_parameter("tS_LongitudinalController", 0.01);
  }
  // print all parameters
  for (std::string name : this->list_parameters({}, 1).names) {
    rclcpp::Parameter param = this->get_parameter(name);
    RCLCPP_DEBUG(
      this->get_logger(), "param name: %s, value: %s", param.get_name().c_str(),
      param.value_to_string().c_str());
  }

  // Bind the callbacks to the timer
  double time = this->get_parameter("tS_LongitudinalController").as_double();
  model_update_timer_ = tam::create_timer(
    this, std::chrono::microseconds(static_cast<uint64_t>(time * 1e6)),
    std::bind(&LongitudinalControllerNode::function_queue_callback, this));

  //   Subscriber
  sub_acceleration_ =
    topic_watchdog_->add_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/core/state/acceleration", tam::ros::get_qos(),
      std::bind(&LongitudinalControllerNode::acceleration_callback, this, _1),
      longitudinal_controller_state_machine_->GetTimeoutFunction(
        LongitudinalControllerStateMachine::Signals::Acceleration),
      timeout_values.value_ms("StateEstimation"));

  sub_omega_engine_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat32Stamped>(
    "/vehicle/sensor/omega_engine_radps", tam::ros::get_qos(),
    std::bind(&LongitudinalControllerNode::omega_engine_callback, this, _1),
    longitudinal_controller_state_machine_->GetTimeoutFunction(
      LongitudinalControllerStateMachine::Signals::Omega_Engine),
    timeout_values.value_ms("Default"));

  sub_gear_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMInt8Stamped>(
    "/vehicle/sensor/gear", tam::ros::get_qos(),
    std::bind(&LongitudinalControllerNode::gear_callback, this, _1),
    longitudinal_controller_state_machine_->GetTimeoutFunction(
      LongitudinalControllerStateMachine::Signals::Gear),
    timeout_values.value_ms("Default"));

  sub_brake_pressure_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
    "/vehicle/sensor/brake_pressure_Pa", tam::ros::get_qos(),
    std::bind(&LongitudinalControllerNode::brake_pressure_callback, this, _1),
    longitudinal_controller_state_machine_->GetTimeoutFunction(
      LongitudinalControllerStateMachine::Signals::Brake_Pressure),
    timeout_values.value_ms("Default"));

  sub_odometry_ = topic_watchdog_->add_subscription<nav_msgs::msg::Odometry>(
    "/core/state/odometry", tam::ros::get_qos(),
    std::bind(&LongitudinalControllerNode::odometry_callback, this, _1),
    longitudinal_controller_state_machine_->GetTimeoutFunction(
      LongitudinalControllerStateMachine::Signals::Odometry),
    timeout_values.value_ms("StateEstimation"));

  sub_long_acc_target_ =
    topic_watchdog_->add_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/core/control/tracking_controller/control_request", tam::ros::get_qos(),
      std::bind(&LongitudinalControllerNode::target_acceleration_callback, this, _1),
      longitudinal_controller_state_machine_->GetTimeoutFunction(
        LongitudinalControllerStateMachine::Signals::Target_Acceleration),
      timeout_values.value_ms("TrackingController"));

  sub_warmup_brake_pressure_ =
    topic_watchdog_->add_subscription<tum_msgs::msg::TUMFloat64PerWheelStamped>(
      "/core/control/warm_up_brake_pressure_pa", tam::ros::get_qos(),
      std::bind(&LongitudinalControllerNode::brake_warmup_pressure_callback, this, _1),
      [this](bool timeout, std::chrono::milliseconds) {
        if (timeout) {
          longitudinal_controller_->set_target_brake_warmup_pressure_Pa(
            tam::types::common::DataPerWheel<double>(0.0));
          // std::cout << "Brake Warmup Pressure Timeout: " << timeout_ms.count() << " ms";
        }
      },
      timeout_values.default_or_ms(""));

  sub_gear_request_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMInt8Stamped>(
    "/core/control/gear_request", tam::ros::get_qos(),
    std::bind(&LongitudinalControllerNode::gear_request_callback, this, _1),
    longitudinal_controller_state_machine_->GetTimeoutFunction(
      LongitudinalControllerStateMachine::Signals::Gear_Request),
    timeout_values.value_ms("Default"));

  sub_slip_control_active_ = topic_watchdog_->add_subscription<tum_msgs::msg::TUMBoolStamped>(
    "/core/control/slip_control_active", tam::ros::get_qos(),
    std::bind(&LongitudinalControllerNode::slip_control_active_callback, this, _1),
    longitudinal_controller_state_machine_->GetTimeoutFunction(
      LongitudinalControllerStateMachine::Signals::Slip_Control_Active),
    timeout_values.value_ms("Default"));

  // Publisher
  ctrl_cmd_pub_ = this->create_publisher<tum_msgs::msg::TUMLongitudinalCmd>(
    "/core/control/longitudinal_controller/longitudinal_request", tam::ros::get_qos());
}
void LongitudinalControllerNode::function_queue_callback()
{
  topic_watchdog_->check_timeouts();
  // First trigger the timeout callback
  // model_update_callback();
  evaluate_diagnostics();
  // Then add the node monitor callback to publish the diagnostics
  node_monitor_->update();
}
void LongitudinalControllerNode::model_update_callback()
{
  // Logging
  callback_start_time = std::chrono::steady_clock::now();

  if (
    longitudinal_controller_state_machine_->GetLongitudinalControllerState() ==
      LongitudinalControllerStateMachine::longitudinal_controller_state_::startup ||
    longitudinal_controller_state_machine_->CheckIsTimeouted(
      LongitudinalControllerStateMachine::Signals::Target_Acceleration)) {
    return;
  }

  cycle_count_++;

  // step controller
  longitudinal_controller_->step();

  // Get a stamp for the published messages
  builtin_interfaces::msg::Time stamp = get_clock()->now();

  // publish ice commands
  tum_msgs::msg::TUMLongitudinalCmd long_out;
  long_out.stamp = stamp;
  tam::types::control::ICECommand ice = longitudinal_controller_->get_ice_commands();
  long_out.brake_pressure_pa.front_left = ice.brake_pressure_Pa.front_left;
  long_out.brake_pressure_pa.front_right = ice.brake_pressure_Pa.front_right;
  long_out.brake_pressure_pa.rear_left = ice.brake_pressure_Pa.rear_left;
  long_out.brake_pressure_pa.rear_right = ice.brake_pressure_Pa.rear_right;
  long_out.gear = ice.gear;
  long_out.throttle = ice.throttle;
  ctrl_cmd_pub_->publish(long_out);

  // Add timing to debug signals
  logger_->log(
    "timing/exec_time_timer_callback_us",
    std::chrono::duration_cast<std::chrono::microseconds>(
      (std::chrono::steady_clock::now() - callback_start_time))
      .count());
  logger_->log(
    "timing/time_since_last_callback_us",
    std::chrono::duration_cast<std::chrono::microseconds>((callback_start_time - last_call_time))
      .count());
  logger_->log("timing/cycle_count", cycle_count_);

  auto state_machine_state = longitudinal_controller_state_machine_->get_diagnostic_state();
  logger_->log("state_machine/state", static_cast<int>(state_machine_state.state));
  int timeout_code = 0;
  for (std::size_t i = 0; i < state_machine_state.timeout_detected.size(); i++) {
    if (state_machine_state.timeout_detected[i]) {
      timeout_code += std::pow(2, i + 1);
    }
  }
  logger_->log("state_machine/timeout_code", timeout_code);
  last_call_time = callback_start_time;
  tsl_publisher_.trigger();
}
void LongitudinalControllerNode::evaluate_diagnostics()
{
  auto state_machine_state = longitudinal_controller_state_machine_->get_diagnostic_state();

  if (
    state_machine_state.state !=
    LongitudinalControllerStateMachine::longitudinal_controller_state_::startup) {
    node_monitor_->initialization_finished();
  }

  node_monitor_->set_message(state_machine_state.message);
  node_monitor_->set_error_lvl(state_machine_state.causing_key, state_machine_state.error_lvl);
  node_monitor_->set_status_code(state_machine_state.state);
}
void LongitudinalControllerNode::acceleration_callback(
  const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Acceleration);

  tam::types::control::AccelerationwithCovariances input;
  input.acceleration_mps2.x = msg->accel.accel.linear.x;
  input.acceleration_mps2.y = msg->accel.accel.linear.y;
  input.acceleration_mps2.z = msg->accel.accel.linear.z;
  input.angular_acceleration_radps2.x = msg->accel.accel.angular.x;
  input.angular_acceleration_radps2.y = msg->accel.accel.angular.y;
  input.angular_acceleration_radps2.z = msg->accel.accel.angular.z;

  longitudinal_controller_->set_feedback_acceleration(input);
}
void LongitudinalControllerNode::omega_engine_callback(
  const tum_msgs::msg::TUMFloat32Stamped::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Omega_Engine);

  drivetrain.omega_engine_radps = msg->data;
  longitudinal_controller_->set_feedback_drivetrain(drivetrain);
}
void LongitudinalControllerNode::gear_callback(const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Gear);

  drivetrain.gear_engaged = msg->data;
  longitudinal_controller_->set_feedback_drivetrain(drivetrain);
}
void LongitudinalControllerNode::brake_pressure_callback(
  const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Brake_Pressure);

  tam::types::common::DataPerWheel<double> input;
  input.front_left = msg->data.front_left;
  input.front_right = msg->data.front_right;
  input.rear_left = msg->data.rear_left;
  input.rear_right = msg->data.rear_right;
  longitudinal_controller_->set_feedback_brake_pressure_Pa(input);
}
void LongitudinalControllerNode::brake_warmup_pressure_callback(
  const tum_msgs::msg::TUMFloat64PerWheelStamped::SharedPtr msg)
{
  tam::types::common::DataPerWheel<double> input;
  input.front_left = msg->data.front_left;
  input.front_right = msg->data.front_right;
  input.rear_left = msg->data.rear_left;
  input.rear_right = msg->data.rear_right;
  longitudinal_controller_->set_target_brake_warmup_pressure_Pa(input);
}
void LongitudinalControllerNode::gear_request_callback(
  const tum_msgs::msg::TUMInt8Stamped::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Gear_Request);
  longitudinal_controller_->set_gear_control_request(msg->data);
}
void LongitudinalControllerNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Odometry);

  tam::types::control::Odometry input;
  input = tam::type::conversions::cpp::Odometry_type_from_msg(msg);

  longitudinal_controller_->set_feedback_odometry(input);
}
void LongitudinalControllerNode::target_acceleration_callback(
  const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Target_Acceleration);

  longitudinal_controller_->set_long_acc_target_mps2(msg->longitudinal.acceleration);

  LongitudinalControllerNode::model_update_callback();
}
void LongitudinalControllerNode::slip_control_active_callback(const tum_msgs::msg::TUMBoolStamped::SharedPtr msg)
{
  longitudinal_controller_state_machine_->SetReceivedOnce(
    LongitudinalControllerStateMachine::Signals::Slip_Control_Active);
  longitudinal_controller_->set_slip_control_active(msg->data);
}
}  // namespace tam::control

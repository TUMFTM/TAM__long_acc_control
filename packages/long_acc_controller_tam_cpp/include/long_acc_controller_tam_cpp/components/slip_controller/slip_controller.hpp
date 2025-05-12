#pragma once
#include <algorithm>
#include <map>
#include <memory>
#include <vector>

#include "long_acc_controller_tam_cpp/components/slip_controller/abs_controlled_wheel.hpp"
#include "long_acc_controller_tam_cpp/components/slip_controller/slip_controller_types.hpp"
#include "long_acc_controller_tam_cpp/components/slip_controller/tc_controlled_wheel.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
//#include "long_acc_controller_tam_cpp/components/slip_controller/signal_verification.hpp"
#include "param_management_cpp/param_manager_composer.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/composer.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
// region Controller Interface
namespace tam::control
{
class SlipController
{
private:
  using Dpw = tam::types::common::DataPerWheel<double>;
  using Bpw = tam::types::common::DataPerWheel<bool>;
  template <typename T>
  using Cyclic_Vector = tam::helpers::control::Cyclic_Vector<T>;
  using time_point = std::chrono::steady_clock::time_point;
  template <typename T>
  using Flank_Detector = tam::helpers::control::Flank_Detector<T>;

  Flank_Detector<int8_t> gear_change_detector = Flank_Detector<int8_t>();
  Flank_Detector<bool> abs_latched_detector = Flank_Detector<bool>();
  Flank_Detector<bool> tc_latched_detector = Flank_Detector<bool>();

  SlipControlInputs slip_control_inputs{};
  SlipControlState slip_control_state{};
  SlipControlParams slip_control_params{};

  tam::helpers::control::FirstOrderLowPass<Dpw> slip_angle_filter{Dpw{0.0}, Dpw{0.7}};
  tam::helpers::control::FirstOrderLowPass<Dpw> slip_rate_filter{Dpw{0.0}, Dpw{0.7}};

  tam::types::common::DataPerWheel<ABSControlledWheel> abs_controlled_wheels{
    ABSControlledWheel{Wheel_Position::Front_Left}, ABSControlledWheel{Wheel_Position::Front_Right},
    ABSControlledWheel{Wheel_Position::Rear_Left}, ABSControlledWheel{Wheel_Position::Rear_Right}};
  tam::types::common::DataPerWheel<TCControlledWheel> tc_controlled_wheels{
    TCControlledWheel{Wheel_Position::Front_Left}, TCControlledWheel{Wheel_Position::Front_Right},
    TCControlledWheel{Wheel_Position::Rear_Left}, TCControlledWheel{Wheel_Position::Rear_Right}};

  tam::types::common::DataPerWheel<AbsTcInputs> abs_inputs;
  tam::types::common::DataPerWheel<AbsTcInputs> tc_inputs;

  // param manager
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_composer_ =
    std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<tam::pmg::MgmtInterface::SharedPtr>{
        abs_controlled_wheels[0].get_param_manager(), abs_controlled_wheels[1].get_param_manager(),
        abs_controlled_wheels[2].get_param_manager(), abs_controlled_wheels[3].get_param_manager(),
        tc_controlled_wheels[0].get_param_manager(), tc_controlled_wheels[1].get_param_manager(),
        tc_controlled_wheels[2].get_param_manager(), tc_controlled_wheels[3].get_param_manager(),
        param_manager_});
  std::size_t previous_param_state_hash = 0;

  // debug container
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  tam::tsl::LoggerComposer::SharedPtr logger_composer_ = std::make_shared<tam::tsl::LoggerComposer>(
    std::vector<tam::tsl::LoggerAccessInterface::SharedPtr>{logger_});

public:
  SlipController()
  {
    declare_and_update_parameters();
    logger_composer_->register_logger(
      abs_controlled_wheels.front_left.get_debug_out(), "ABS/front_left/");
    logger_composer_->register_logger(
      abs_controlled_wheels.front_right.get_debug_out(), "ABS/front_right/");
    logger_composer_->register_logger(
      abs_controlled_wheels.rear_left.get_debug_out(), "ABS/rear_left/");
    logger_composer_->register_logger(
      abs_controlled_wheels.rear_right.get_debug_out(), "ABS/rear_right/");
    logger_composer_->register_logger(
      tc_controlled_wheels.front_left.get_debug_out(), "TC/front_left/");
    logger_composer_->register_logger(
      tc_controlled_wheels.front_right.get_debug_out(), "TC/front_right/");
    logger_composer_->register_logger(
      tc_controlled_wheels.rear_left.get_debug_out(), "TC/rear_left/");
    logger_composer_->register_logger(
      tc_controlled_wheels.rear_right.get_debug_out(), "TC/rear_right/");
  }
  void declare_and_update_parameters()
  {
    slip_control_params.abs_is_activated =
      param_manager_
        ->declare_and_get_value("ABS.IsActivated", false, tam::pmg::ParameterType::BOOL, "")
        .as_bool();
    slip_control_params.abs_operation_mode = static_cast<OperationMode>(
      param_manager_
        ->declare_and_get_value(
          "ABS.OperationMode", static_cast<int>(OperationMode::front_rear_split),
          tam::pmg::ParameterType::INTEGER, "")
        .as_int());
    slip_control_params.abs_min_activation_velocity =
      param_manager_
        ->declare_and_get_value(
          "ABS.MinActivationVelocity", 5.0, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.abs_min_activation_acceleration =
      param_manager_
        ->declare_and_get_value(
          "ABS.MinActivationAcceleration", -5.0, tam::pmg::ParameterType::DOUBLE,
          "Disables the slip control for low decelerations. Safety feature!")
        .as_double();
    slip_control_params.abs_cooldown_gear_change =
      param_manager_
        ->declare_and_get_value(
          "ABS.ActivationTimeAfterGearshift", 0.8, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.tc_is_activated =
      param_manager_
        ->declare_and_get_value("TC.IsActivated", false, tam::pmg::ParameterType::BOOL, "")
        .as_bool();
    slip_control_params.tc_operation_mode =
      static_cast<OperationMode>(param_manager_
                                   ->declare_and_get_value(
                                     "TC.OperationMode", static_cast<int>(OperationMode::rear_only),
                                     tam::pmg::ParameterType::INTEGER, "")
                                   .as_int());
    slip_control_params.tc_min_activation_velocity =
      param_manager_
        ->declare_and_get_value(
          "TC.MinActivationVelocity", 0.0, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.tc_cooldown_gear_change =
      param_manager_
        ->declare_and_get_value(
          "TC.ActivationTimeAfterGearshift", 0.8, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.P_VDC_MaxSlipThrottleCut =
      param_manager_
        ->declare_and_get_value(
          "P_VDC_MaxSlipThrottleCut", 25.0, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.slip_angle_filter_pole =
      param_manager_
        ->declare_and_get_value("SlipAngleFilterPole", 0.7, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.slip_rate_filter_pole =
      param_manager_
        ->declare_and_get_value("SlipRateFilterPole", 0.7, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.tS =
      param_manager_
        ->declare_and_get_value(
          "tS_LongitudinalController", 0.01, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    slip_control_params.lookahead_horizon =
      param_manager_
        ->declare_and_get_value("LookaheadHorizon", 0.05, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();

    // update filters
    slip_rate_filter.set_tf_pole(Dpw{slip_control_params.slip_rate_filter_pole});
    slip_angle_filter.set_tf_pole(Dpw{slip_control_params.slip_angle_filter_pole});

    previous_param_state_hash = param_manager_->get_state_hash();
  }
  // Non controller specific
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() { return param_manager_composer_; }
  // Step method
  void step()
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash) {
      declare_and_update_parameters();
    }
    // ABS
    // compute abs tc input structs
    set_allowed();

    for (size_t i = 0; i < abs_controlled_wheels.size(); i++) {
      abs_inputs[i].long_fx = slip_control_inputs.long_fx;
      abs_inputs[i].slip = slip_control_inputs.slip_input_abs[i];
      abs_inputs[i].slip_valid = slip_control_inputs.slip_valid;
      abs_inputs[i].odometry = slip_control_inputs.odometry;
      abs_inputs[i].brake_pressure = slip_control_inputs.target_brake_pressure[i];
      abs_inputs[i].slip_lookahead = slip_control_inputs.slip_lookahead_abs[i];
      abs_inputs[i].slip_angle = slip_control_inputs.slip_angle_abs[i];
      abs_inputs[i].slip_rate = slip_control_inputs.slip_rate_abs[i];
      abs_controlled_wheels[i].set_abs_inputs(abs_inputs[i]);
      abs_controlled_wheels[i].step();
    }

    for (size_t i = 0; i < tc_controlled_wheels.size(); i++) {
      tc_inputs[i].long_fx = slip_control_inputs.long_fx;
      tc_inputs[i].slip = slip_control_inputs.slip_input_tc[i];
      tc_inputs[i].slip_valid = slip_control_inputs.slip_valid;
      tc_inputs[i].odometry = slip_control_inputs.odometry;
      tc_inputs[i].brake_pressure = slip_control_inputs.target_brake_pressure[i];
      tc_inputs[i].slip_angle = slip_control_inputs.slip_angle_tc[i];
      tc_controlled_wheels[i].set_tc_inputs(tc_inputs[i]);
      tc_controlled_wheels[i].step();
    }

    slip_control_state.abs_latched =
      abs_controlled_wheels[0].get_is_latched() || abs_controlled_wheels[1].get_is_latched() ||
      abs_controlled_wheels[2].get_is_latched() || abs_controlled_wheels[3].get_is_latched();
    slip_control_state.tc_latched =
      slip_control_params.tc_operation_mode != OperationMode::rear_only
        ? tc_controlled_wheels[0].get_is_latched() || tc_controlled_wheels[1].get_is_latched() ||
            tc_controlled_wheels[2].get_is_latched() || tc_controlled_wheels[3].get_is_latched()
        : tc_controlled_wheels[2].get_is_latched() || tc_controlled_wheels[3].get_is_latched();

    // brake pressure target
    if (slip_control_state.abs_latched && slip_control_inputs.long_fx < 0.0) {
      for (size_t i = 0; i < abs_controlled_wheels.size(); i++) {
        slip_control_state.brake_pressure_target_bar[i] =
          abs_controlled_wheels[i].get_target_brake_pressure();
      }
    } else if (slip_control_state.tc_latched && slip_control_inputs.long_fx > 0.0) {
      for (size_t i = 0; i < tc_controlled_wheels.size(); i++) {
        slip_control_state.brake_pressure_target_bar[i] =
          tc_controlled_wheels[i].get_target_brake_pressure();
      }
    } else {
      slip_control_state.brake_pressure_target_bar = slip_control_inputs.target_brake_pressure;
    }
    if (
      abs_latched_detector.check_and_update(slip_control_state.abs_latched) ||
      tc_latched_detector.check_and_update(slip_control_state.tc_latched)) {
      slip_control_state.activation_distance = 0.0;
      slip_control_state.last_call_time = std::chrono::steady_clock::now();
    }
    if (slip_control_state.abs_latched || slip_control_state.tc_latched) {
      double delta_t = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - slip_control_state.last_call_time)
                         .count();
      slip_control_state.last_call_time = std::chrono::steady_clock::now();
      slip_control_state.activation_distance =
        slip_control_state.activation_distance +
        std::sqrt(
          std::pow(slip_control_inputs.odometry.velocity_mps.x * delta_t, 2) +
          std::pow(slip_control_inputs.odometry.velocity_mps.y * delta_t, 2) +
          std::pow(slip_control_inputs.odometry.velocity_mps.z * delta_t, 2));
    }
    // Additional safety if tc intervention is not
    slip_control_state.throttle_target =
      slip_control_inputs.throttle_request * (std::max(
                                                {slip_control_inputs.slip_input_tc.front_left,
                                                 slip_control_inputs.slip_input_tc.front_right,
                                                 slip_control_inputs.slip_input_tc.rear_left,
                                                 slip_control_inputs.slip_input_tc.rear_right}) <=
                                              slip_control_params.P_VDC_MaxSlipThrottleCut);

    logger_->log("activation_distance", slip_control_state.activation_distance);
    logger_->log("ABS/state", slip_control_state.abs_latched);
    logger_->log("TC/state", slip_control_state.tc_latched);
    logger_->log("slip_rate", slip_control_state.slip_rate);
    logger_->log("slip_rate_filtered", slip_control_state.slip_rate_filtered);
  }
  void set_allowed()
  {
    if (gear_change_detector.check_and_update(slip_control_inputs.gear)) {
      slip_control_state.time_gear_changed = std::chrono::steady_clock::now();
    }
    Bpw abs_allowed{
      (slip_control_params.abs_is_activated &&  // No Lint
       (slip_control_inputs.odometry.velocity_mps.x >
        slip_control_params.abs_min_activation_velocity))};
    abs_allowed =
      abs_allowed &&
      Bpw{}.from_front_and_rear(
        true, std::chrono::duration<double>(
                std::chrono::steady_clock::now() - slip_control_state.time_gear_changed) >
                  std::chrono::duration<double>(slip_control_params.abs_cooldown_gear_change) ||
                abs_controlled_wheels[2].get_is_latched() ||
                abs_controlled_wheels[3].get_is_latched());

    Bpw tc_allowed{
      slip_control_params.tc_is_activated &&  // No Lint
      (slip_control_inputs.odometry.velocity_mps.x >
       slip_control_params.tc_min_activation_velocity)};
    tc_allowed =
      tc_allowed &&
      Bpw{}.from_front_and_rear(
        slip_control_params.tc_operation_mode != OperationMode::rear_only,
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - slip_control_state.time_gear_changed) >
            std::chrono::duration<double>(slip_control_params.tc_cooldown_gear_change) ||
          tc_controlled_wheels[2].get_is_latched() || tc_controlled_wheels[3].get_is_latched());
    for (size_t i = 0; i < abs_inputs.size(); i++) {
      abs_inputs[i].allowed = abs_allowed[i];
      tc_inputs[i].allowed = tc_allowed[i];
    }
  }
  // Inputs
  void set_feedback_slips(Dpw slip, Dpw slip_angle)
  {
    // calculate slip rate
    slip_control_state.slip_rate = (slip - slip_control_state.slip) / slip_control_params.tS;
    slip_control_state.slip_rate_filtered = slip_rate_filter.step(slip_control_state.slip_rate);
    slip_control_state.slip = slip;
    // calculate lookahead slip
    Dpw slip_lookahead =
      slip + slip_control_state.slip_rate_filtered * slip_control_params.lookahead_horizon;

    // filter slip angle and set as input
    Dpw filtered_slip_angle = std::abs(slip_angle_filter.step(slip_angle));

    if (slip_control_params.abs_operation_mode == OperationMode::individual) {
      slip_control_inputs.slip_input_abs = slip;
      slip_control_inputs.slip_angle_abs = filtered_slip_angle;
      slip_control_inputs.slip_lookahead_abs = slip_lookahead;
      slip_control_inputs.slip_rate_abs = slip_control_state.slip_rate_filtered;

    } else if (slip_control_params.abs_operation_mode == OperationMode::front_rear_split) {
      slip_control_inputs.slip_input_abs = Dpw{}.from_front_and_rear(
        std::min(slip.front_left, slip.front_right), std::min(slip.rear_left, slip.rear_right));
      slip_control_inputs.slip_angle_abs = Dpw{}.from_front_and_rear(
        std::max(filtered_slip_angle.front_left, filtered_slip_angle.front_right),
        std::max(filtered_slip_angle.rear_left, filtered_slip_angle.rear_right));
      slip_control_inputs.slip_lookahead_abs = Dpw{}.from_front_and_rear(
        std::min(slip_lookahead.front_left, slip_lookahead.front_right),
        std::min(slip_lookahead.rear_left, slip_lookahead.rear_right));
      slip_control_inputs.slip_rate_abs = Dpw{}.from_front_and_rear(
        slip_lookahead.front_left < slip_lookahead.front_right
          ? slip_control_state.slip_rate_filtered.front_left
          : slip_control_state.slip_rate_filtered.front_right,
        slip_lookahead.rear_left < slip_lookahead.rear_right
          ? slip_control_state.slip_rate_filtered.rear_left
          : slip_control_state.slip_rate_filtered.rear_right);
    } else if (slip_control_params.abs_operation_mode == OperationMode::all_together) {
      slip_control_inputs.slip_input_abs =
        Dpw{std::min({slip.front_left, slip.front_right, slip.rear_left, slip.rear_right})};
      slip_control_inputs.slip_angle_abs = Dpw{std::max(
        {filtered_slip_angle.front_left, filtered_slip_angle.front_right,
         filtered_slip_angle.rear_left, filtered_slip_angle.rear_right})};
      slip_control_inputs.slip_lookahead_abs = Dpw{std::min(
        {slip_lookahead.front_left, slip_lookahead.front_right, slip_lookahead.rear_left,
         slip_lookahead.rear_right})};
      auto slip_lookahaead_array = slip_lookahead.to_array();
      slip_control_inputs.slip_rate_abs = Dpw{slip_control_state.slip_rate_filtered[std::distance(
        slip_lookahaead_array.begin(),
        std::min_element(slip_lookahaead_array.begin(), slip_lookahaead_array.end()))]};
    }
    if (slip_control_params.tc_operation_mode == OperationMode::individual) {
      slip_control_inputs.slip_input_tc = slip;
      slip_control_inputs.slip_angle_tc = filtered_slip_angle;
      slip_control_inputs.slip_lookahead_tc = slip_lookahead;
    } else if (slip_control_params.tc_operation_mode == OperationMode::front_rear_split) {
      slip_control_inputs.slip_input_tc = Dpw{}.from_front_and_rear(
        std::max(slip.front_left, slip.front_right), std::max(slip.rear_left, slip.rear_right));
      slip_control_inputs.slip_angle_tc = Dpw{}.from_front_and_rear(
        std::max(filtered_slip_angle.front_left, filtered_slip_angle.front_right),
        std::max(filtered_slip_angle.rear_left, filtered_slip_angle.rear_right));
      slip_control_inputs.slip_lookahead_tc = Dpw{}.from_front_and_rear(
        std::max(slip_lookahead.front_left, slip_lookahead.front_right),
        std::max(slip_lookahead.rear_left, slip_lookahead.rear_right));
    } else if (slip_control_params.tc_operation_mode == OperationMode::all_together) {
      slip_control_inputs.slip_input_tc =
        Dpw{std::max({slip.front_left, slip.front_right, slip.rear_left, slip.rear_right})};
      slip_control_inputs.slip_angle_tc = Dpw{std::max(
        {filtered_slip_angle.front_left, filtered_slip_angle.front_right,
         filtered_slip_angle.rear_left, filtered_slip_angle.rear_right})};
      slip_control_inputs.slip_lookahead_tc = Dpw{std::max(
        {slip_lookahead.front_left, slip_lookahead.front_right, slip_lookahead.rear_left,
         slip_lookahead.rear_right})};
    } else if (slip_control_params.tc_operation_mode == OperationMode::rear_only) {
      slip_control_inputs.slip_input_tc =
        Dpw{}.from_front_and_rear(0.0, std::max(slip.rear_left, slip.rear_right));
      slip_control_inputs.slip_angle_tc = Dpw{}.from_front_and_rear(
        0.0, std::max(filtered_slip_angle.rear_left, filtered_slip_angle.rear_right));
      slip_control_inputs.slip_lookahead_tc = Dpw{}.from_front_and_rear(
        0.0, std::max(slip_lookahead.rear_left, slip_lookahead.rear_right));
    }
  }
  void set_feedback_slip_valid(bool slip_valid) { slip_control_inputs.slip_valid = slip_valid; }
  void set_feedback_long_fx(double long_fx) { slip_control_inputs.long_fx = long_fx; }
  void set_feedback_target_brake_presure(Dpw target_brake_pressure)
  {
    slip_control_inputs.target_brake_pressure = target_brake_pressure;
  }
  void set_feedback_throttle_request(double throttle_request)
  {
    slip_control_inputs.throttle_request = throttle_request;
  }
  void set_feedback_gear(int8_t gear) { slip_control_inputs.gear = gear; }
  void set_feedback_odometry(tam::types::control::Odometry odometry)
  {
    slip_control_inputs.odometry = odometry;
  }
  void set_operation_mode(const tam::types::control::AutowareOperationMode &) {}
  // Outputs
  tam::types::common::DataPerWheel<double> get_break_pressure_target_bar() const
  {
    return slip_control_state.brake_pressure_target_bar;
  }
  double get_throttle_request() const { return slip_control_state.throttle_target; }
  SlipControlStatus get_status() const
  {
    return SlipControlStatus(slip_control_state.abs_latched, slip_control_state.tc_latched);
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_composer_; }
};
}  // namespace tam::control
// endregion

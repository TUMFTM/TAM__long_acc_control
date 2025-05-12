#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#include "long_acc_controller_tam_cpp/components/slip_controller/abs_params.hpp"
#include "long_acc_controller_tam_cpp/components/slip_controller/slip_controller_types.hpp"
#include "param_management_cpp/base.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
class ABSControlledWheel
{
public:
  explicit ABSControlledWheel(Wheel_Position wheel_position) : params(wheel_position) {}
  void step()
  {
    if (params.param_changed()) {
      params.declare_and_update_parameters();
      update_moving_average_length();
    }
    calculate_moving_averages();
    calculate_reduction_factor();
    update_slip_target_reduction();
    calculate_slip_error();
    Wheel_States new_wheel_state = transitions(abs_state.wheel_state);
    if (new_wheel_state != Wheel_States::Not_Changed) {
      abs_state.wheel_state = new_wheel_state;
    }
    abs_state.long_fx_output =
      abs_state.eps * abs_state.reduction_factor * abs_state.long_fx_latched;
    abs_state.long_fx_output = std::max(abs_state.long_fx_output, abs_inputs.long_fx);
    if (abs_state.wheel_state == Wheel_States::Not_Latched) {
      abs_state.long_fx_output = abs_inputs.long_fx;
    }
    update_safety_feature();
    convert_fx_to_brake_pressure();
    log_debug_values();
  }
  void set_abs_inputs(AbsTcInputs abs_inputs_) { abs_inputs = abs_inputs_; }
  double get_target_brake_pressure() const { return abs_state.brake_pressure_output; }
  bool get_is_latched() const
  {
    if (abs_state.wheel_state >= Wheel_States::Reduction_Phase) {
      return true;
    }
    return false;
  }
  bool is_latched()
  {
    return (
      (abs_inputs.slip_lookahead <= abs_state.slip_target_reduction * params.target_slip) &&
      (params.min_long_force - 100 > abs_inputs.long_fx) && abs_inputs.allowed);
  }
  bool is_not_latched()
  {
    return ((params.min_long_force < abs_inputs.long_fx) || !abs_inputs.allowed);
  }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const
  {
    return params.get_param_manager();
  }
  void log_debug_values()
  {
    logger_->log("slip", abs_inputs.slip);
    logger_->log("slip_target", params.target_slip);
    logger_->log("slip_lookahead", abs_inputs.slip_lookahead);
    logger_->log("wheel_state", static_cast<int>(abs_state.wheel_state));
    logger_->log("allowed", abs_inputs.allowed);
    logger_->log("long_fx_latched", abs_state.long_fx_latched);
    logger_->log("long_fx_output", abs_state.long_fx_output);
    logger_->log("brake_pressure_latched", abs_state.brake_pressure_latched);
    logger_->log("brake_pressure_output", abs_state.brake_pressure_output);
    logger_->log("eps", abs_state.eps);
    logger_->log("long_fx_output_average", abs_state.long_fx_output_average);
    logger_->log("long_fx_input_average", abs_state.long_fx_input_average);
    logger_->log("safety_feature_active", abs_state.safety_feature_is_active);
    logger_->log("reduction_factor", abs_state.reduction_factor);
    logger_->log("slip_target_reduction", abs_state.slip_target_reduction);
    logger_->log("slip_rate", abs_inputs.slip_rate);
    logger_->log("slip_error", abs_state.slip_error);
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_; }

private:
  AbsTcInputs abs_inputs{};
  AbsState abs_state{};
  AbsParams params;
  void update_moving_average_length()
  {
    abs_state.long_fx_output_vector.resize(params.MovingAverageFxWindowLength);
    abs_state.long_fx_input_vector.resize(params.MovingAverageFxWindowLength);
  }
  void reduce_target()
  {
    abs_state.eps =
      std::clamp(abs_state.eps - abs_state.slip_error * params.red_phase_deduction, 0.0, 1.0);
  }
  void increase_target()
  {
    abs_state.eps =
      std::clamp(abs_state.eps + abs_state.slip_error * params.inc_phase_addition, 0.0, 1.0);
  }
  Wheel_States transitions(Wheel_States state)
  {
    switch (state) {
      case Wheel_States::Not_Latched:
        if (is_latched()) {
          abs_state.eps = params.initial_value_eps_red_phase;
          abs_state.long_fx_latched = abs_inputs.long_fx;
          abs_state.brake_pressure_latched = abs_inputs.brake_pressure;
          abs_state.v_latched = abs_inputs.odometry.velocity_mps.x;
          return Wheel_States::Reduction_Phase;
        }
        return Wheel_States::Not_Changed;
      case Wheel_States::Reduction_Phase:
        if (is_not_latched()) return Wheel_States::Not_Latched;
        if (
          abs_inputs.slip_lookahead <
          abs_state.slip_target_reduction * params.target_red_phase_to_red_phase) {
          if (abs_inputs.slip_rate < 0.0) {
            reduce_target();
          }
          return Wheel_States::Reduction_Phase;
        }
        if (
          abs_inputs.slip_lookahead >=
          abs_state.slip_target_reduction * params.target_red_phase_to_inc_phase) {
          abs_state.eps = std::max(params.initial_value_eps_inc_phase, abs_state.eps);
          return Wheel_States::Increase_Phase;
        }
        return Wheel_States::Not_Changed;
      case Wheel_States::Increase_Phase:
        if (is_not_latched()) return Wheel_States::Not_Latched;
        if (
          abs_inputs.slip_lookahead <
          abs_state.slip_target_reduction * params.target_inc_phase_to_red_phase) {
          abs_state.eps = std::min(abs_state.eps, params.initial_value_eps_red_phase);
          return Wheel_States::Reduction_Phase;
        }
        if (
          abs_inputs.slip_lookahead >=
          abs_state.slip_target_reduction * params.target_inc_phase_to_inc_phase) {
          increase_target();
          return Wheel_States::Increase_Phase;
        }
        return Wheel_States::Not_Changed;
      default:
        std::cout << "[ABS]: Error: Invalid state!" << std::endl;
        return Wheel_States::Not_Latched;
    }
  }
  void update_safety_feature()
  {
    abs_state.safety_feature_is_active = false;
    if (
      (abs_state.long_fx_output_average >
       params.fx_safety_threshold_factor_ * abs_state.long_fx_input_average) &&
      (abs_state.wheel_state > Wheel_States::Not_Latched)) {
      abs_state.long_fx_output =
        std::min(params.fx_safety_threshold_factor_ * abs_inputs.long_fx, abs_state.long_fx_output);
      abs_state.safety_feature_is_active = true;
    }
  }
  void calculate_moving_averages()
  {
    abs_state.long_fx_output_vector.insert(
      abs_state.long_fx_output > 0.0 ? 0.0 : abs_state.long_fx_output);
    abs_state.long_fx_output_average =
      (std::reduce(abs_state.long_fx_output_vector.begin(), abs_state.long_fx_output_vector.end()) /
       params.MovingAverageFxWindowLength);
    abs_state.long_fx_input_vector.insert(abs_inputs.long_fx > 0.0 ? 0.0 : abs_inputs.long_fx);
    abs_state.long_fx_input_average =
      (std::reduce(abs_state.long_fx_input_vector.begin(), abs_state.long_fx_input_vector.end()) /
       params.MovingAverageFxWindowLength);
  }
  void calculate_reduction_factor()
  {
    if (abs_state.wheel_state == Wheel_States::Not_Latched) {
      abs_state.reduction_factor = 0.9;
    } else {
      double factor_fz_aero =
        -0.5 * params.air_density * params.cross_track_area * params.lift_coeff;
      double Fz_static = params.mass * 9.81;
      abs_state.reduction_factor =
        0.9 * std::clamp(
                1.0 - (factor_fz_aero * (std::pow(abs_state.v_latched, 2) -
                                         std::pow(abs_inputs.odometry.velocity_mps.x, 2))) /
                        (Fz_static + factor_fz_aero * std::pow(abs_state.v_latched, 2)),
                0.33, 1.0);
    }
  }
  void update_slip_target_reduction()
  {
    abs_state.slip_target_reduction =
      (1.0 - std::clamp(abs_inputs.slip_angle / params.slip_angle_max, 0.0, 1.0) *
               params.slip_target_reduction_factor);
  }
  void convert_fx_to_brake_pressure()
  {
    double brake_pressure_reduction =
      params.fx_to_brake_pressure_factor *
      (abs_state.long_fx_output - abs_state.reduction_factor * abs_state.long_fx_latched);
    abs_state.brake_pressure_output = std::max(
      abs_state.reduction_factor * abs_state.brake_pressure_latched - brake_pressure_reduction,
      0.0);
    abs_state.brake_pressure_output =
      std::min(abs_inputs.brake_pressure, abs_state.brake_pressure_output);
  }
  void calculate_slip_error()
  {
    abs_state.slip_error = std::clamp(
      std::abs(
        (abs_state.slip_target_reduction * params.target_slip - abs_inputs.slip_lookahead) /
        (abs_state.slip_target_reduction * params.target_slip)),
      1 / 10.0, 1.0);
  }
  //  Integrations
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
};
}  // namespace tam::control

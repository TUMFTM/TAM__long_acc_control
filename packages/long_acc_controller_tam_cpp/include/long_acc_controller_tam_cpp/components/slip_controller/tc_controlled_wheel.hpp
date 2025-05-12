#pragma once
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "long_acc_controller_tam_cpp/components/slip_controller/slip_controller_types.hpp"
#include "long_acc_controller_tam_cpp/components/slip_controller/tc_params.hpp"
#include "param_management_cpp/base.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
class TCControlledWheel
{
public:
  explicit TCControlledWheel(Wheel_Position wheel_position) : tc_params(wheel_position) {}
  void step()
  {
    // params
    if (tc_params.param_changed()) {
      tc_params.declare_and_update_parameters();
      slip_filter.set_tf_pole(tc_params.slip_filter_pole);
    }
    update_slip_target_reduction();
    calculate_slip_error();
    Wheel_States new_wheel_state = transitions(tc_state.wheel_state);
    if (new_wheel_state != Wheel_States::Not_Changed) {
      tc_state.wheel_state = new_wheel_state;
    }
    tc_state.long_fx_output = tc_state.eps * tc_state.target_adjustment_factor_ * tc_inputs.long_fx;
    convert_fx_to_brake_pressure();
    if (tc_state.wheel_state == Wheel_States::Not_Latched || tc_inputs.long_fx < 0.0) {
      tc_state.long_fx_output = tc_inputs.long_fx;
      tc_state.target_brake_pressure_output = tc_inputs.brake_pressure;
    }
    if (tc_state.current_slip_ >= tc_state.slip_target_reduction * tc_params.slip_target) {
      tc_state.last_time_threshold_exceeded = std::chrono::steady_clock::now();
    }
    log_debug_values();
  }
  void set_tc_inputs(AbsTcInputs tc_inputs_)
  {
    tc_inputs = tc_inputs_;
    tc_state.current_slip_ = slip_filter.step(tc_inputs.slip);
    if (tc_state.wheel_state != Wheel_States::Not_Latched) {
      tc_state.target_adjustment_factor_ =
        tc_state.long_fx_input_latched / tc_inputs.long_fx +
        (1.0 - tc_state.long_fx_input_latched / tc_inputs.long_fx) / (std::max(1e-2, tc_state.eps));
    }
  }
  double get_target_brake_pressure() const { return tc_state.target_brake_pressure_output; }
  bool get_is_latched() const
  {
    if (tc_state.wheel_state >= Wheel_States::Reduction_Phase) {
      return true;
    }
    return false;
  }
  bool is_latched()
  {
    return (
      tc_state.current_slip_ >= tc_state.slip_target_reduction * tc_params.slip_target &&
      tc_inputs.allowed && tc_inputs.long_fx > 0.0);
  }
  bool is_not_latched()
  {
    return (
      (std::chrono::duration<double>(
        std::chrono::steady_clock::now() - tc_state.last_time_threshold_exceeded)) >
        std::chrono::duration<double>(tc_params.deactivation_duration) ||
      !tc_inputs.allowed);
  }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const
  {
    return tc_params.get_param_manager();
  }
  void log_debug_values()
  {
    logger_->log("slip", tc_state.current_slip_);
    logger_->log("slip_angle", tc_inputs.slip_angle);
    logger_->log("slip_unfiltered", tc_inputs.slip);
    logger_->log("slip_target", tc_params.slip_target);
    logger_->log("wheel_state", static_cast<int>(tc_state.wheel_state));
    logger_->log("allowed", tc_inputs.allowed);
    logger_->log("long_fx_input_latched", tc_state.long_fx_input_latched);
    logger_->log("long_fx_output", tc_state.long_fx_output);
    logger_->log("target_brake_pressure_latched", tc_state.target_brake_pressure_latched);
    logger_->log("target_brake_pressure_output", tc_state.target_brake_pressure_output);
    logger_->log("eps", tc_state.eps);
    logger_->log("slip_target_reduction", tc_state.slip_target_reduction);
    logger_->log("target_adjustment_factor", tc_state.target_adjustment_factor_);
    logger_->log("slip_error", tc_state.slip_error);
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_; }

private:
  TCParams tc_params;
  TcState tc_state{};
  AbsTcInputs tc_inputs{};
  tam::helpers::control::FirstOrderLowPass<double> slip_filter{};
  Wheel_Position wheel_position_{};
  void reduce_target()
  {
    tc_state.eps =
      std::clamp(tc_state.eps - tc_state.slip_error * tc_params.red_phase_deduction, 0.0, 1.0);
  }
  void increase_target()
  {
    tc_state.eps =
      std::clamp(tc_state.eps + tc_state.slip_error * tc_params.inc_phase_addition, 0.0, 1.0);
  }

  Wheel_States transitions(Wheel_States state)
  {
    switch (state) {
      case Wheel_States::Not_Latched:
        if (is_latched()) {
          tc_state.eps = tc_params.eps_reset;
          tc_state.target_brake_pressure_latched = tc_inputs.brake_pressure;
          tc_state.target_adjustment_factor_ = 1.0;
          tc_state.long_fx_input_latched = tc_inputs.long_fx;
          return Wheel_States::Reduction_Phase;
        }
        return Wheel_States::Not_Latched;
      case Wheel_States::Reduction_Phase:
        if (is_not_latched()) return Wheel_States::Not_Latched;
        if (
          tc_state.current_slip_ >
          tc_state.slip_target_reduction * tc_params.target_red_phase_to_red_phase) {
          reduce_target();
          return Wheel_States::Reduction_Phase;
        }
        if (
          tc_state.current_slip_ <=
          tc_state.slip_target_reduction * tc_params.target_red_phase_to_inc_phase) {
          increase_target();
          return Wheel_States::Increase_Phase;
        }
        return Wheel_States::Not_Changed;
      case Wheel_States::Increase_Phase:
        if (is_not_latched()) return Wheel_States::Not_Latched;
        if (
          tc_state.current_slip_ >
          tc_state.slip_target_reduction * tc_params.target_inc_phase_to_red_phase) {
          tc_state.eps = std::min(tc_state.eps, tc_params.eps_reset);
          return Wheel_States::Reduction_Phase;
        }
        if (
          tc_state.current_slip_ <=
          tc_state.slip_target_reduction * tc_params.target_inc_phase_to_inc_phase) {
          increase_target();
          return Wheel_States::Increase_Phase;
        }
        return Wheel_States::Not_Changed;
      default:
        std::cout << "[TC]: Error: Invalid state!" << std::endl;
        return Wheel_States::Not_Latched;
    }
  }
  void update_slip_target_reduction()
  {
    tc_state.slip_target_reduction =
      (1.0 - std::clamp(tc_inputs.slip_angle / tc_params.slip_angle_max, 0.0, 1.0) *
               tc_params.slip_target_reduction_factor);
  }
  void calculate_slip_error()
  {
    tc_state.slip_error = std::clamp(
      std::abs(
        (tc_state.slip_target_reduction * tc_params.slip_target - tc_state.current_slip_) /
        (tc_state.slip_target_reduction * tc_params.slip_target)),
      1 / 10.0, 1.0);
  }
  void convert_fx_to_brake_pressure()
  {
    tc_state.target_brake_pressure_output = (tc_state.long_fx_output <= tc_inputs.long_fx) *
                                            (-1.0) * tc_params.fx_to_brake_pressure_factor *
                                            (tc_state.long_fx_output - tc_inputs.long_fx);
    tc_state.target_brake_pressure_output =
      std::max(tc_inputs.brake_pressure, tc_state.target_brake_pressure_output);
  }
  // Integrations
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
};
}  // namespace tam::control

#pragma once

#include "long_acc_controller_tam_cpp/components/slip_controller/slip_controller_types.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
class AbsParams
{
public:
  Wheel_Position wheel_position_{Wheel_Position::Front_Left};
  double brake_bias_front{};
  double r_BrakePadsMeanLeverFrRe_m__2{};
  double n_BrakePadsNumberFrRe__2{};
  double mue_BrakePadsFrRe_kinetic__2{};
  double d_BrakeActuatorBoreFrRe_m__2{};
  int MovingAverageFxWindowLength{};
  double initial_value_eps_red_phase{};
  double initial_value_eps_inc_phase{};
  double red_phase_deduction{};
  double target_red_phase_to_red_phase{};
  double target_red_phase_to_inc_phase{};
  double target_inc_phase_to_red_phase{};
  double target_inc_phase_to_inc_phase{};
  double inc_phase_addition{};
  double target_slip{};
  double fx_safety_threshold_factor_{};
  double fx_to_brake_pressure_factor{};
  double tyreradius_front{};
  double tyreradius_rear{};
  double min_long_force{};
  double air_density{};
  double cross_track_area{};
  double lift_coeff{};
  double mass{};
  double slip_angle_max{};
  double slip_target_reduction_factor{};
  std::size_t previous_param_state_hash = 0;
  AbsParams(Wheel_Position wheel_position) : wheel_position_(wheel_position)
  {
    declare_and_update_parameters();
  }
  void declare_and_update_parameters()
  {
    auto decl = [this](std::string name, double val) {
      return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    };
    tyreradius_front = decl("tires.front_left.radius.radius_20mps", 0.293475);
    tyreradius_rear = decl("tires.rear_left.radius.radius_20mps", 0.3074348);
    brake_bias_front = decl("P_VDC_brake_bias_front", 0.55);
    r_BrakePadsMeanLeverFrRe_m__2 = decl("vehicle.brake.pad_mean_radius", 0.134);
    n_BrakePadsNumberFrRe__2 = decl("vehicle.brake.pad_number", 1.0);
    mue_BrakePadsFrRe_kinetic__2 = decl("vehicle.brake.friction_coeff", 0.35);
    d_BrakeActuatorBoreFrRe_m__2 = decl("vehicle.brake.piston_diameter", 0.0798);
    air_density = decl("vehicle.aero.air_density", 1.22);
    cross_track_area = decl("vehicle.aero.cross_track_area", 1.0);
    lift_coeff = decl("vehicle.aero.lift_coeff", -1.65);
    mass = decl("vehicle.mass.total", 800.0);
    min_long_force = decl("ABS.MinLongForce", -4000.0);
    slip_angle_max = decl("ABS.slip_angle_max", 0.07);
    slip_target_reduction_factor = decl("ABS.slip_target_reduction_factor", 0.3);
    MovingAverageFxWindowLength =
      param_manager_
        ->declare_and_get_value(
          "ABS.MovingAverageFxWindowLength", 50, tam::pmg::ParameterType::INTEGER, "")
        .as_int();
    previous_param_state_hash = param_manager_->get_state_hash();
    if (
      (wheel_position_ == Wheel_Position::Front_Left) ||
      (wheel_position_ == Wheel_Position::Front_Right)) {
      initial_value_eps_red_phase = decl("ABS.FrontInitialValueEpsRedPhase", 0.6);
      initial_value_eps_inc_phase = decl("ABS.FrontInitialValueEpsIncPhase", 0.2);
      red_phase_deduction = decl("ABS.FrontRedPhaseDeduction", 0.1);
      target_red_phase_to_red_phase = decl("ABS.FrontRedPhaseToRedPhase", -10.0);
      target_red_phase_to_inc_phase = decl("ABS.FrontRedPhaseToIncPhase", -10.0);
      target_inc_phase_to_red_phase = decl("ABS.FrontIncPhaseToRedPhase", -10.0);
      target_inc_phase_to_inc_phase = decl("ABS.FrontIncPhaseToIncPhase", -10.0);
      inc_phase_addition = decl("ABS.FrontIncPhaseAddition", 0.25);
      target_slip = decl("ABS.FrontTargetSlip", -10.0);
      fx_safety_threshold_factor_ = decl("ABS.FrontFxSafetyThresholdFactor", 0.7);
      fx_to_brake_pressure_factor =
        1.0 /
        (2.0 * r_BrakePadsMeanLeverFrRe_m__2 * n_BrakePadsNumberFrRe__2 *
         mue_BrakePadsFrRe_kinetic__2 * M_PI * std::pow(d_BrakeActuatorBoreFrRe_m__2 / 2.0, 2) *
         tam::constants::pascal_per_bar) *
        tyreradius_front * brake_bias_front;
    } else if (
      (wheel_position_ == Wheel_Position::Rear_Left) ||
      (wheel_position_ == Wheel_Position::Rear_Right)) {
      initial_value_eps_red_phase = decl("ABS.RearInitialValueEpsRedPhase", 0.95);
      initial_value_eps_inc_phase = decl("ABS.RearInitialValueEpsIncPhase", 0.7);
      red_phase_deduction = decl("ABS.RearRedPhaseDeduction", 0.1);
      target_red_phase_to_red_phase = decl("ABS.RearRedPhaseToRedPhase", -12.0);
      target_red_phase_to_inc_phase = decl("ABS.RearRedPhaseToIncPhase", -5.0);
      target_inc_phase_to_red_phase = decl("ABS.RearIncPhaseToRedPhase", -10.0);
      target_inc_phase_to_inc_phase = decl("ABS.RearIncPhaseToIncPhase", -5.0);
      inc_phase_addition = decl("ABS.RearIncPhaseAddition", 0.25);
      target_slip = decl("ABS.RearTargetSlip", -10.0);
      fx_safety_threshold_factor_ = decl("ABS.RearFxSafetyThresholdFactor", 0.7);
      fx_to_brake_pressure_factor =
        1.0 /
        (2.0 * r_BrakePadsMeanLeverFrRe_m__2 * n_BrakePadsNumberFrRe__2 *
         mue_BrakePadsFrRe_kinetic__2 * M_PI * std::pow(d_BrakeActuatorBoreFrRe_m__2 / 2.0, 2) *
         tam::constants::pascal_per_bar) *
        tyreradius_rear * (1.0 - brake_bias_front);
    }
  }
  bool param_changed() { return param_manager_->get_state_hash() != previous_param_state_hash; }
  tam::pmg::MgmtInterface::SharedPtr get_param_manager() const { return param_manager_; }

private:
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
};
}  // namespace tam::control
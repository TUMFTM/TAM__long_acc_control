#pragma once

#include "long_acc_controller_tam_cpp/components/slip_controller/slip_controller_types.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
class TCParams
{
public:
  Wheel_Position wheel_position_{Wheel_Position::Front_Left};
  double brake_bias_front{};
  double r_BrakePadsMeanLeverFrRe_m__2{};
  double n_BrakePadsNumberFrRe__2{};
  double mue_BrakePadsFrRe_kinetic__2{};
  double d_BrakeActuatorBoreFrRe_m__2{};
  double slip_target{};
  double eps_reset{};
  double red_phase_deduction{};
  double inc_phase_addition{};
  double target_red_phase_to_red_phase{};
  double target_red_phase_to_inc_phase{};
  double target_inc_phase_to_red_phase{};
  double target_inc_phase_to_inc_phase{};
  double fx_to_brake_pressure_factor{};
  double tyreradius_front{};
  double tyreradius_rear{};
  double slip_filter_pole{};
  double slip_target_reduction_factor{};
  double slip_angle_max{};
  double deactivation_duration{};
  std::size_t previous_param_state_hash = 0;
  TCParams(Wheel_Position wheel_position) : wheel_position_(wheel_position)
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
    slip_filter_pole = decl("TC.slip_filter_pole", 0.85);
    slip_target_reduction_factor = decl("TC.slip_target_reduction_factor", 0.5);
    slip_angle_max = decl("TC.slip_angle_max", 0.07);
    deactivation_duration = decl("TC.DeactivationDuration", 2.0);
    previous_param_state_hash = param_manager_->get_state_hash();
    if (
      (wheel_position_ == Wheel_Position::Front_Left) ||
      (wheel_position_ == Wheel_Position::Front_Right)) {
      slip_target = decl("TC.FrontThresholdLatched", 10.0);
      eps_reset = decl("TC.FrontEpsReset", 0.95);
      red_phase_deduction = decl("TC.FrontRedPhaseDeduction", 0.2);
      inc_phase_addition = decl("TC.FrontIncPhaseAddition", 0.05);
      target_red_phase_to_red_phase = decl("TC.FrontRedPhaseToRedPhase", 12.0);
      target_red_phase_to_inc_phase = decl("TC.FrontRedPhaseToIncPhase", 5.0);
      target_inc_phase_to_red_phase = decl("TC.FrontIncPhaseToRedPhase", 10.0);
      target_inc_phase_to_inc_phase = decl("TC.FrontIncPhaseToIncPhase", 5.0);
      fx_to_brake_pressure_factor =
        1.0 /
        (2.0 * r_BrakePadsMeanLeverFrRe_m__2 * n_BrakePadsNumberFrRe__2 *
         mue_BrakePadsFrRe_kinetic__2 * M_PI * std::pow(d_BrakeActuatorBoreFrRe_m__2 / 2.0, 2) *
         tam::constants::pascal_per_bar) *
        tyreradius_front * brake_bias_front;
    } else if (
      (wheel_position_ == Wheel_Position::Rear_Left) ||
      (wheel_position_ == Wheel_Position::Rear_Right)) {
      slip_target = decl("TC.RearThresholdLatched", 12.0);
      eps_reset = decl("TC.RearEpsReset", 0.95);
      red_phase_deduction = decl("TC.RearRedPhaseDeduction", 0.1);
      inc_phase_addition = decl("TC.RearIncPhaseAddition", 0.05);
      target_red_phase_to_red_phase = decl("TC.RearRedPhaseToRedPhase", 12.0);
      target_red_phase_to_inc_phase = decl("TC.RearRedPhaseToIncPhase", 5.0);
      target_inc_phase_to_red_phase = decl("TC.RearIncPhaseToRedPhase", 10.0);
      target_inc_phase_to_inc_phase = decl("TC.RearIncPhaseToIncPhase", 5.0);
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
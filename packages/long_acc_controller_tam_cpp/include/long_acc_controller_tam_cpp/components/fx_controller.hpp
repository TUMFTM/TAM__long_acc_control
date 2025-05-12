#pragma once
#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <vector>

#include "controller_helpers_cpp/engine_map.hpp"
#include "long_acc_controller_tam_cpp/components/slip_controller/slip_controller_types.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_helpers_cpp/constants.hpp"
#include "tum_helpers_cpp/geometry/geometry.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
// region Controller Interface
namespace tam::control
{
class FxController
{
public:
  FxController()
  {
    declare_and_update_parameters();
    engine_map_ = std::make_shared<tam::control::engine::EngineMap>(param_manager_);
  }
  // Non controller specific
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const { return param_manager_; }
  // Step method
  void step()
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash) {
      declare_and_update_parameters();
      this->engine_map_->declare_and_update_parameters();
    }
    tam::types::control::EngineTorques engine_torques = get_engine_torques(omega_engine_radps_);
    // Calculate the brake pressure target
    tam::types::common::DataPerWheel<double> break_pressure_target_bar_without_warmup =
      calculate_brake_pressure_target(
        current_gear_, v_mps_, engine_torques.T_min_Nm, Fx_request_N_);
    break_pressure_target_bar_ =
      std::max(break_pressure_target_bar_without_warmup, target_pressure_brake_warmup_bar_);
    logger_->log("brake_pressure_target", break_pressure_target_bar_);
    // Determine the amount of additional braking to warmup the brake disc
    double fx_additional_brake_warmup =
      get_additional_fx_brake_warmup_N(break_pressure_target_bar_without_warmup);
    // Calculate the throttle
    double T_target_Nm = calculate_torque_target(
      Fx_request_N_ + fx_additional_brake_warmup, omega_engine_radps_, current_gear_);
    double throttle_pos_target =
      calculate_throttle_target(engine_torques, T_target_Nm, omega_engine_radps_);
    throttle_command_ = calculate_throttle_command(
      long_acc_target_mps2_, long_acc_mps2_, v_mps_, Fx_request_N_ + fx_additional_brake_warmup,
      throttle_pos_target, gear_request_, current_gear_);
  }
  // Inputs
  void set_feedback_drivetrain(const tam::types::control::DriveTrainFeedback & drivetrain_feedback)
  {
    current_gear_ = drivetrain_feedback.gear_engaged;
    omega_engine_radps_ = drivetrain_feedback.omega_engine_radps;
  }
  void set_Fx_request_N(const double & Fx_request_N) { Fx_request_N_ = Fx_request_N; }
  void set_gear_request(const int8_t & gear_request) { gear_request_ = gear_request; }
  void set_long_acc_target_mps2(const double & long_acc_target_mps2)
  {
    long_acc_target_mps2_ = long_acc_target_mps2;
  }
  void set_feedback_acceleration(
    const tam::types::control::AccelerationwithCovariances & accelerations)
  {
    long_acc_mps2_ = accelerations.acceleration_mps2.x;
  }
  void set_feedback_odometry(const tam::types::control::Odometry & odometry)
  {
    v_mps_ = tam::helpers::geometry::euclidean_norm(odometry.velocity_mps);
  }
  void set_slip_control_status(const SlipControlStatus & slip_control_status)
  {
    slip_control_status_ = slip_control_status;
  }
  void set_operation_mode(const tam::types::control::AutowareOperationMode & operation_mode)
  {
    switch (operation_mode) {
      case tam::types::control::AutowareOperationMode::AUTONOMOUS:
        EnableDriver_ = true;
        EnableEmergency_ = false;
        break;
      case tam::types::control::AutowareOperationMode::STOP:
        EnableDriver_ = true;
        EnableEmergency_ = true;
        break;
      case tam::types::control::AutowareOperationMode::REMOTE:
        EnableDriver_ = false;
        EnableEmergency_ = false;
        break;
      default:
        break;
    }
  }
  void set_target_brake_warmup_pressure_Pa(
    const tam::types::common::DataPerWheel<double> & brake_pressure_Pa)
  {
    target_pressure_brake_warmup_bar_ = brake_pressure_Pa / tam::constants::pascal_per_bar;
  }
  // Outputs
  double get_throttle_command() const { return throttle_command_; }
  tam::types::common::DataPerWheel<double> get_break_pressure_target() const
  {
    return break_pressure_target_bar_;
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_; }

private:
  // parameters
  struct
  {
    double tire_radius_front_m_20mps;
    double tire_radius_rear_m_20mps;
    double tS;
    double EngineLimit_Ts;
    double n_idle_rpm;
    double p_torque_rear;
    double c_diff_ratio;
    double c_diff_efficiency;
    double f_diff_ratio;
    double f_diff_efficiency;
    double r_diff_ratio;
    double r_diff_efficiency;
    double n_limit_rpm;
    double throttle_ax_target_gain;
    double throttle_FB_error_Ts;
    double throttleKp;
    double throttleKi;
    double throttleKd;
    double throttle_min_speed_mps;
    double throttle_FB_active;
    double throttle_shift_wait_time_s;
    double throttle_saturation_pos;
    double throttle_saturation_neg;
    double MaxThrottleLowSpeed;
    double MaxThrottleLowSpeedVelocity_mps;
    double torque_output_scale;
    double drivetrain_efficiency;
    double min_vel_mps;
    double brake_bias_front;
    double r_BrakePadsMeanLeverFrRe_m__2;
    double n_BrakePadsNumberFrRe__2;
    double mue_BrakePadsFrRe_kinetic__2;
    double d_BrakeActuatorBoreFrRe_m__2;
    double throttle_pos_Ts;
    std::vector<double> i_gearset_table;
    std::vector<double> throttle_adaptive_gain_breakpoints;
    std::vector<double> throttle_adaptive_gains;
  } p_;
  // param manager
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash = 0;

  // debug container
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  std::shared_ptr<tam::control::engine::EngineMap> engine_map_{};

  // buffer
  int8_t current_gear_{};
  int8_t gear_request_{};
  double omega_engine_radps_{};
  double Fx_request_N_{};
  double long_acc_target_mps2_{};
  double long_acc_mps2_{};
  double throttle_command_{};
  double v_mps_{};
  tam::types::common::DataPerWheel<double> target_pressure_brake_warmup_bar_{};
  tam::types::common::DataPerWheel<double> break_pressure_target_bar_{};

  SlipControlStatus slip_control_status_{};
  bool EnableDriver_{}, EnableEmergency_{};

  // filters
  tam::helpers::control::FirstOrderLowPass<double> T_max_filter_{};
  tam::helpers::control::FirstOrderLowPass<double> T_min_filter_{};
  tam::helpers::control::FirstOrderLowPass<double> throttle_pos_filter_{};

  // throttle control
  bool throttle_saturation_triggered_{false};
  tam::helpers::control::PIDControl<double> pid_throttle{};
  // time gearshift request
  bool time_gearshift_request_set{false};
  std::chrono::steady_clock::time_point time_last_gearshift_request_{};
  void declare_and_update_parameters()
  {
    auto decl = [this](std::string name, double val) {
      return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    };
    // param declaration
    p_.tire_radius_front_m_20mps = decl("tires.front_left.radius.radius_20mps", 0.293475);
    p_.tire_radius_rear_m_20mps = decl("tires.rear_left.radius.radius_20mps", 0.3074348);
    p_.tS = decl("tS_LongitudinalController", 0.01);
    p_.EngineLimit_Ts = decl("EngineLimit_Ts", 0.05);
    p_.n_idle_rpm = 60.0 / (2.0 * M_PI) * decl("vehicle.engine.rev_idle", 157.0);
    p_.p_torque_rear = decl("P_VDC_p_torque_rear", 1.0);
    p_.c_diff_ratio = decl("vehicle.drivetrain.transmission_ratio", 3.0);
    p_.c_diff_efficiency = decl("P_VDC_POWTR__DIFFERENTIAL__c_diff_efficiency", 1.0);
    p_.f_diff_ratio = decl("P_VDC_POWTR__DIFFERENTIAL__f_diff_ratio", 1.0);
    p_.f_diff_efficiency = decl("P_VDC_POWTR__DIFFERENTIAL__f_diff_efficiency", 1.0);
    p_.r_diff_ratio = decl("P_VDC_POWTR__DIFFERENTIAL__r_diff_ratio", 1.0);
    p_.r_diff_efficiency = decl("P_VDC_POWTR__DIFFERENTIAL__r_diff_efficiency", 1.0);
    p_.n_limit_rpm = 60.0 / (2.0 * M_PI) * decl("vehicle.engine.rev_max", 712.0);
    p_.throttle_ax_target_gain = decl("P_VDC_throttle_ax_target_gain", 0.95);
    p_.throttle_FB_error_Ts = decl("P_VDC_throttle_FB_error_Ts", 0.04);
    p_.throttleKp = decl("P_VDC_ThrottleKp", 0.01);
    p_.throttleKi = decl("P_VDC_ThrottleKi", 0.1);
    p_.throttleKd = decl("P_VDC_ThrottleKd", 0.0);
    p_.throttle_min_speed_mps = decl("P_VDC_Throttle_min_speed_mps", 8.0);
    p_.throttle_FB_active = decl("P_VDC_Throttle_FB_active", 1.0);
    p_.throttle_shift_wait_time_s = decl("P_VDC_throttle_shift_wait_time_s", 1.0);
    p_.throttle_saturation_pos = decl("P_VDC_throttle_saturation_pos", 0.1);
    p_.throttle_saturation_neg = decl("P_VDC_throttle_saturation_neg", -0.3);
    p_.MaxThrottleLowSpeed = decl("P_VDC_MaxThrottleLowSpeed", 0.25);
    p_.MaxThrottleLowSpeedVelocity_mps = decl("P_VDC_MaxThrottleLowSpeedVelocity_mps", 5.0);
    p_.torque_output_scale = decl("P_VDC_torque_output_scale", 1.0);
    p_.drivetrain_efficiency = decl("vehicle.drivetrain.drivetrain_efficiency", 0.97);
    p_.min_vel_mps = decl("P_VDC_MinVelSlipCalc_mps", 3.0);
    p_.brake_bias_front = decl("P_VDC_brake_bias_front", 0.55);
    p_.r_BrakePadsMeanLeverFrRe_m__2 = decl("vehicle.brake.pad_mean_radius", 0.134);
    p_.n_BrakePadsNumberFrRe__2 = decl("vehicle.brake.pad_number", 1.0);
    p_.mue_BrakePadsFrRe_kinetic__2 = decl("vehicle.brake.friction_coeff", 0.35);
    p_.d_BrakeActuatorBoreFrRe_m__2 = decl("vehicle.brake.piston_diameter", 0.0798);
    p_.throttle_pos_Ts = decl("throttle_pos_Ts", 0.8);
    p_.i_gearset_table =
      param_manager_
        ->declare_and_get_value(
          "vehicle.drivetrain.gear_ratios",
          std::vector<double>({0.0, 2.9167, 1.875, 1.3809, 1.1154, 0.96, 0.8889}),
          tam::pmg::ParameterType::DOUBLE_ARRAY, "")
        .as_double_array();
    p_.throttle_adaptive_gain_breakpoints =
      param_manager_
        ->declare_and_get_value(
          "P_VDC_throttle_adaptive_gain_breakpoints",
          std::vector<double>({-3.0, -1.0, -0.5, 0.0, 0.5, 3.0}),
          tam::pmg::ParameterType::DOUBLE_ARRAY, "")
        .as_double_array();
    p_.throttle_adaptive_gains =
      param_manager_
        ->declare_and_get_value(
          "P_VDC_throttle_adaptive_gains", std::vector<double>({0.3, 1.5, 1.5, 1.0, 0.7, 0.3}),
          tam::pmg::ParameterType::DOUBLE_ARRAY, "")
        .as_double_array();
    previous_param_state_hash = param_manager_->get_state_hash();

    // engine torque filters
    double torque_pole = std::exp(-p_.tS / p_.EngineLimit_Ts);
    T_max_filter_.set_tf_pole(torque_pole);
    T_min_filter_.set_tf_pole(torque_pole);
    throttle_pos_filter_.set_tf_pole(p_.throttle_pos_Ts);

    // throttle pid
    pid_throttle.set_params(
      std::exp(-p_.tS / p_.throttle_FB_error_Ts), p_.throttleKp, p_.throttleKi, p_.throttleKd,
      p_.tS, p_.throttle_saturation_neg, p_.throttle_saturation_pos);
  }
  double get_additional_fx_brake_warmup_N(
    tam::types::common::DataPerWheel<double> break_pressure_target_bar_without_warmup)
  {
    // Determine the amount of additional braking to warmup the tire
    tam::types::common::DataPerWheel<double> additonal_warmup_pressure_bar =
      std::max((break_pressure_target_bar_ - break_pressure_target_bar_without_warmup), 0.0);
    tam::types::common::DataPerWheel<double> fx_additional_brake_warmup_per_wheel =
      additonal_warmup_pressure_bar * tam::constants::pascal_per_bar * 0.25 *
      std::pow(p_.d_BrakeActuatorBoreFrRe_m__2, 2) * M_PI * p_.n_BrakePadsNumberFrRe__2 *
      p_.mue_BrakePadsFrRe_kinetic__2 * p_.r_BrakePadsMeanLeverFrRe_m__2 /
      tam::types::common::DataPerWheel<double>::from_front_and_rear(
        p_.tire_radius_front_m_20mps, p_.tire_radius_rear_m_20mps);

    double return_val = fx_additional_brake_warmup_per_wheel.front_left +
                        fx_additional_brake_warmup_per_wheel.front_right +
                        fx_additional_brake_warmup_per_wheel.rear_left +
                        fx_additional_brake_warmup_per_wheel.rear_right;

    logger_->log("brake_warmup/add_warmup_pressure_bar", additonal_warmup_pressure_bar);
    logger_->log("brake_warmup/fx_engine_additional", return_val);
    return return_val;
  }
  tam::types::control::EngineTorques get_engine_torques(const double omega_engine_radps)
  {
    // output
    tam::types::control::EngineTorques engine_torques;
    // get values from engine map
    double T_max_Nm_in =
      p_.torque_output_scale * engine_map_->lookup_engine_map_Nm(omega_engine_radps, 1.0);
    double T_min_Nm_in = engine_map_->lookup_engine_map_Nm(omega_engine_radps, 0.0);
    // Transfer functions
    engine_torques.T_max_Nm = T_max_filter_.step(T_max_Nm_in);
    engine_torques.T_min_Nm = T_min_filter_.step(T_min_Nm_in);

    return engine_torques;
  }
  double calculate_torque_target(
    const double F_x_request_N, const double omega_engine_radps, const int8_t gear_engaged)
  {
    // Calculate torque_request = IDLE + requested
    // IDLE torque
    double n_delta_rpm = p_.n_idle_rpm - omega_engine_radps * 60.0 / (2.0 * M_PI);
    double T_IDLE_Nm = (n_delta_rpm >= 0) ? n_delta_rpm : 0.0;
    // Requested engine torque
    double T_Nm = F_x_request_N * p_.tire_radius_rear_m_20mps;
    double T_target_Nm{};
    if (gear_engaged != 0) {
      T_target_Nm = T_Nm / (p_.c_diff_ratio * p_.c_diff_efficiency *
                            ((1.0 - p_.p_torque_rear) * p_.f_diff_ratio * p_.f_diff_efficiency +
                             p_.p_torque_rear * p_.r_diff_ratio * p_.r_diff_efficiency) *
                            p_.i_gearset_table[gear_engaged - 1] * p_.drivetrain_efficiency);
    } else {
      // use first gear even if in neutral
      T_target_Nm = T_Nm / (p_.c_diff_ratio * p_.c_diff_efficiency *
                            ((1.0 - p_.p_torque_rear) * p_.f_diff_ratio * p_.f_diff_efficiency +
                             p_.p_torque_rear * p_.r_diff_ratio * p_.r_diff_efficiency) *
                            p_.i_gearset_table[gear_engaged] * p_.drivetrain_efficiency);
    }
    // Engine Limiter
    n_delta_rpm = p_.n_limit_rpm - omega_engine_radps * 60.0 / (2.0 * M_PI);
    double cutoff =
      (F_x_request_N > 0.0) ? (1 / (exp(-n_delta_rpm) + 1)) + ((-1) / (exp(n_delta_rpm) + 1)) : 1.0;
    cutoff = std::max(0.0, std::min(cutoff, 1.0));
    // Add up Torques and limit
    T_target_Nm = cutoff * (T_target_Nm + T_IDLE_Nm);

    return T_target_Nm;
  }
  double calculate_throttle_target(
    const tam::types::control::EngineTorques engine_torques, const double T_target_Nm,
    const double omega_engine_radps)
  {
    // Limit Torque to maximum engine values for debug
    double T_engine_Nm =
      std::max(engine_torques.T_min_Nm, std::min(engine_torques.T_max_Nm, T_target_Nm));

    // calculate throttle position target
    // interpolate throttle position for torque request above throttle support value
    double throttle_pos_target = throttle_pos_filter_.step(
      engine_map_->lookup_throttle_value(omega_engine_radps, T_target_Nm));
    // guarantee a throttle application value[0, 1]
    throttle_pos_target = std::max(0.0, std::min(throttle_pos_target, 1.0));

    // debug
    logger_->log("T_engine", T_engine_Nm);
    logger_->log("p_throttle_pos_target", throttle_pos_target);

    return throttle_pos_target;
  }
  double calculate_throttle_command(
    const double ax_target_mps2, const double ax_mps2, const double v_mps,
    const double F_x_request_N, const double throttle_pos_target, const int8_t gear_request,
    const int8_t current_gear)
  {
    // use ax error for throttle pid control
    double ax_error_mps2 = p_.throttle_ax_target_gain * ax_target_mps2 - ax_mps2;
    double throttle_error_gain = tam::helpers::numerical::interp(
      ax_error_mps2, p_.throttle_adaptive_gain_breakpoints, p_.throttle_adaptive_gains);
    throttle_error_gain = std::max(throttle_error_gain, p_.throttle_adaptive_gains.front());
    double ax_error_adaptive_mps2 = throttle_error_gain * ax_error_mps2;

    // Check for active gearshift request
    if (gear_request != current_gear) {
      if (!time_gearshift_request_set) {
        time_last_gearshift_request_ = std::chrono::steady_clock::now();
        time_gearshift_request_set = true;
      }
    } else {
      time_gearshift_request_set = false;
    }

    // PID logic
    bool controller_switch = v_mps >= p_.throttle_min_speed_mps && p_.throttle_FB_active;
    bool pid_switch = throttle_pos_target <= 1e-10 || !controller_switch ||
                      std::chrono::duration_cast<std::chrono::seconds>(
                        (std::chrono::steady_clock::now() - time_last_gearshift_request_))
                          .count() < p_.throttle_shift_wait_time_s;

    bool slip_control_active = slip_control_status_.abs_latched || slip_control_status_.tc_latched;

    bool integrator_switch = EnableDriver_ && throttle_pos_target - 1.0 < 1e-10 &&
                             throttle_pos_target > 0.0 && controller_switch && !slip_control_active;

    // PID step
    tam::helpers::control::PIDFeedback throttle_fb = pid_throttle.step(
      ax_error_adaptive_mps2, EnableDriver_, !pid_switch, integrator_switch,
      pid_switch || slip_control_active, EnableDriver_ && !pid_switch);

    // saturate feedback
    double throttle_pos_feedback =
      std::max(
        p_.throttle_saturation_neg, std::min(throttle_fb.feedback, p_.throttle_saturation_pos)) *
      (!slip_control_active);
    throttle_saturation_triggered_ = std::abs(throttle_pos_feedback - throttle_fb.feedback) > 1e-10;

    // add + saturate throttle position
    double throttle_pos =
      controller_switch ? std::max(0.0, std::min(1.0, throttle_pos_target + throttle_pos_feedback))
                        : throttle_pos_target;

    // this helps with braking and imprecisions with the engine drag torque calculation. Ensures
    // braking only at low speeds.
    throttle_pos = !(F_x_request_N < 0.0 && v_mps < p_.min_vel_mps) ? throttle_pos : 0.0;
    // clip throttle for low speed to avoid spinning
    throttle_pos = v_mps >= p_.MaxThrottleLowSpeedVelocity_mps
                     ? throttle_pos
                     : std::min(p_.MaxThrottleLowSpeed, throttle_pos);
    // clip throttle for emergency
    throttle_pos = EnableEmergency_ ? 0.0 : throttle_pos;

    // debug
    logger_->log("Throttle_FB_p", throttle_fb.feedback_p);
    logger_->log("Throttle_FB_i", throttle_fb.feedback_i);
    logger_->log("Throttle_FB_d", throttle_fb.feedback_d);
    logger_->log("Throttle_FB", throttle_pos_feedback);
    logger_->log("ax_error_mps2", ax_error_mps2);
    logger_->log("ax_error_adaptive_mps2", ax_error_adaptive_mps2);

    return throttle_pos;
  }
  tam::types::common::DataPerWheel<double> calculate_brake_pressure_target(
    const int8_t gear_engaged, double v_mps, double T_min_Nm, double F_x_request_N)
  {
    // Calculate braking force caused by engine
    double gear_ratio = gear_engaged != 0
                          ? (p_.c_diff_ratio * p_.c_diff_efficiency *
                             ((1.0 - p_.p_torque_rear) * p_.f_diff_ratio * p_.f_diff_efficiency +
                              p_.p_torque_rear * p_.r_diff_ratio * p_.r_diff_efficiency) *
                             p_.i_gearset_table[gear_engaged - 1] * p_.drivetrain_efficiency) /
                              p_.tire_radius_rear_m_20mps
                          : 0.0;
    double BrakeForce_N = v_mps > p_.min_vel_mps ? T_min_Nm * gear_ratio : 0.0;

    // Get brake pressure target from force
    double force_to_brake_pressure_bar_per_N =
      p_.tire_radius_front_m_20mps /
      (2.0 * p_.r_BrakePadsMeanLeverFrRe_m__2 * p_.n_BrakePadsNumberFrRe__2 *
       p_.mue_BrakePadsFrRe_kinetic__2 * M_PI * std::pow(p_.d_BrakeActuatorBoreFrRe_m__2 / 2.0, 2) *
       tam::constants::pascal_per_bar);

    // Get effective brake pressure targets (on rear axle additional engine force)
    double brake_pressure_target_rear_bar =
      force_to_brake_pressure_bar_per_N *
      std::max(0.0, -((1.0 - p_.brake_bias_front) * F_x_request_N - BrakeForce_N));

    double brake_pressure_target_front_bar =
      force_to_brake_pressure_bar_per_N *
      std::max(
        0.0, std::min(-(F_x_request_N - BrakeForce_N), -F_x_request_N * p_.brake_bias_front));

    tam::types::common::DataPerWheel<double> brake_pressure_target_per_wheel_bar{
      tam::types::common::DataPerWheel<double>{}.from_front_and_rear(
        brake_pressure_target_front_bar, brake_pressure_target_rear_bar)};

    return brake_pressure_target_per_wheel_bar;
  }
};
}  // namespace tam::control
// endregion

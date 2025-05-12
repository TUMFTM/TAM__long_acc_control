#pragma once
#include <cmath>

#include <algorithm>
#include <map>
#include <memory>
#include <string>

#include "controller_helpers_cpp/helpers.hpp"
#include "long_acc_controller_tam_cpp/components/slip_controller/slip_controller_types.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_helpers_cpp/constants.hpp"
#include "tum_helpers_cpp/geometry/geometry.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::control
{
class AxController
{
private:
  // parameters
  struct
  {
    double tS;
    double LongAccFB_Ts;
    double LongAccFB_d_Ts;
    double LongAccFB_turbo_Ts;
    double DriveForceMax_N;
    double DriveForceMin_N;
    double FxMaxSlope_Nps;
    double NegativeFxStandstill;
    double vehiclemass_kg;
    double LongAccKp;
    double LongAccKd;
    double LongAcc_LimFb_N;
    double LongAccTurboComp;
    double rotational_inertia_factor;
    double drag_coefficient;
    double lift_coefficient;
    double A_VehicleReference_m2;
    double Crr_RollingResistance;
    double roh_air;
  } p_;
  // param manager
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash = 0;

  // debug container
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  // buffer
  double feedback_ax_buffer_{};
  double velocity_buffer_{};
  double LongAcc_target_mps2_{};
  double LongAcc_error_old_mps2{};
  double LongAcc_target_old_mps2{};
  double Fx_LongControlRequest_old{};
  bool EnableEmergency_{};
  bool EnableDriver_{};
  double Fx_LongControlRequest_N{};
  SlipControlStatus slip_control_status_{};

  // filter
  tam::helpers::control::FirstOrderLowPass<double> fb_filter_{0.0, 0.0};
  tam::helpers::control::FirstOrderLowPass<double> d_filter_{0.0, 0.0};
  tam::helpers::control::FirstOrderLowPass<double> turbo_filter_{0.0, 0.0};
  void declare_and_update_parameters()
  {
    auto decl = [this](std::string name, double val) {
      return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    };
    p_.tS = decl("tS_LongitudinalController", 0.01);
    p_.LongAccFB_Ts = decl("P_VDC_LongAccFB_Ts", 0.07);
    p_.LongAccFB_d_Ts = decl("P_VDC_LongAccFB_d_Ts", 0.02);
    p_.LongAccFB_turbo_Ts = decl("P_VDC_LongAccFB_turbo_Ts", 0.08);
    p_.DriveForceMax_N = decl("DriveForceMax_N", 6000.0);
    p_.DriveForceMin_N = decl("DriveForceMin_N", -35000.0);
    p_.FxMaxSlope_Nps = decl("P_VDC_FxMaxSlope_Nps", 200000.0);
    p_.NegativeFxStandstill = decl("P_VDC_NegativeFxStandstill", -2000.0);
    p_.vehiclemass_kg = decl("vehicle.mass.total", 800.0);
    p_.LongAccKp = decl("P_VDC_LongAccKp", 100.0);
    p_.LongAccKd = decl("P_VDC_LongAccKd", 0.5);
    p_.LongAcc_LimFb_N = decl("P_VDC_LongAcc_LimFb_N", 1000.0);
    p_.LongAccTurboComp = decl("P_VDC_LongAccTurboComp", 0.0);
    p_.rotational_inertia_factor = decl("rotational_inertia_factor", 1.0);
    p_.drag_coefficient = decl("vehicle.aero.drag_coeff", 0.725);
    p_.lift_coefficient = decl("vehicle.aero.lift_coeff", -1.65);
    p_.A_VehicleReference_m2 = decl("vehicle.aero.cross_track_area", 1.0);
    p_.Crr_RollingResistance = decl("vehicle.rolling_resistance.coeff", 0.025);
    p_.roh_air = decl("vehicle.aero.air_density", 1.22);
    previous_param_state_hash = param_manager_->get_state_hash();

    // filter
    fb_filter_.set_tf_pole(std::exp(-p_.tS / p_.LongAccFB_Ts));
    d_filter_.set_tf_pole(std::exp(-p_.tS / p_.LongAccFB_d_Ts));
    turbo_filter_.set_tf_pole(std::exp(-p_.tS / p_.LongAccFB_turbo_Ts));
  }

public:
  AxController() { declare_and_update_parameters(); }
  // Non controller specific
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const { return param_manager_; };
  // Step method
  void step()
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash) {
      declare_and_update_parameters();
    }
    // region Feedback
    double LongAcc_error_mps2 = (LongAcc_target_mps2_ - feedback_ax_buffer_);
    double LongAcc_deriv_error = (LongAcc_error_mps2 - LongAcc_error_old_mps2) / p_.tS;
    // proportional controller part
    double LongAcc_FB_p = LongAcc_error_mps2 * p_.LongAccKp;
    LongAcc_FB_p = std::max(std::min(LongAcc_FB_p, p_.LongAcc_LimFb_N), -p_.LongAcc_LimFb_N);
    // derivative part
    double LongAcc_FB_d = LongAcc_deriv_error * p_.LongAccKd;
    // filter derivative part
    LongAcc_FB_d = d_filter_.step(LongAcc_FB_d);
    // Fred's Turbo Lag Compensator™
    double LongAcc_FB_turbo_N = (LongAcc_target_mps2_ - LongAcc_target_old_mps2) / p_.tS;
    LongAcc_FB_turbo_N = turbo_filter_.step(LongAcc_FB_turbo_N) * p_.LongAccTurboComp;

    // only enable when...
    if (!(velocity_buffer_ > 10 && LongAcc_target_mps2_ > 0 && LongAcc_FB_turbo_N > 0)) {
      LongAcc_FB_turbo_N = 0;
    }

    double LongAcc_FB_N = LongAcc_FB_p + LongAcc_FB_d + LongAcc_FB_turbo_N;
    // transfer function for feedback command
    LongAcc_FB_N = fb_filter_.step(LongAcc_FB_N);
    // endregion

    // update buffers
    LongAcc_error_old_mps2 = LongAcc_error_mps2;
    LongAcc_target_old_mps2 = LongAcc_target_mps2_;

    // region Feedforward
    double FF_LongAcc = LongAcc_target_mps2_ * p_.vehiclemass_kg;
    double FF_LongAcc_N = FF_LongAcc * p_.rotational_inertia_factor;

    double FF_Aero_N =
      0.5 * pow(velocity_buffer_, 2) * p_.drag_coefficient * p_.A_VehicleReference_m2 * p_.roh_air;
    double FF_RollingResistance_N = (0.5 * pow(velocity_buffer_, 2) * (-p_.lift_coefficient) *
                                       p_.A_VehicleReference_m2 * p_.roh_air +
                                     p_.vehiclemass_kg * tam::constants::g_earth) *
                                    (p_.Crr_RollingResistance) * 2;
    // endregion

    // total force request
    bool slip_control_active = slip_control_status_.abs_latched || slip_control_status_.tc_latched;
    Fx_LongControlRequest_N =
      LongAcc_FB_N * (!slip_control_active) + FF_LongAcc_N + FF_Aero_N + FF_RollingResistance_N;

    // Output checks
    double drive_force_max_N = EnableEmergency_ ? 0.0 : p_.DriveForceMax_N;
    double delta_force_max_N = p_.FxMaxSlope_Nps * p_.tS;
    Fx_LongControlRequest_N =
      !EnableDriver_
        ? p_.NegativeFxStandstill
        : std::min(
            Fx_LongControlRequest_old + delta_force_max_N,
            std::max(Fx_LongControlRequest_old - delta_force_max_N, Fx_LongControlRequest_N));
    Fx_LongControlRequest_old = Fx_LongControlRequest_N;
    Fx_LongControlRequest_N =
      std::min(drive_force_max_N, std::max(p_.DriveForceMin_N, Fx_LongControlRequest_N));

    // debug signal
    logger_->log("RequestLongForce_N", Fx_LongControlRequest_N);
    logger_->log("Target_ax_cpp", LongAcc_target_mps2_);
    logger_->log("FB_LongAcc_p_cpp", LongAcc_FB_p);
    logger_->log("FB_LongAcc_d_cpp", LongAcc_FB_d);
    logger_->log("FB_LongAcc_turbo_cpp", LongAcc_FB_turbo_N);
    logger_->log("FB_LongAcc_cpp", LongAcc_FB_N);
    logger_->log("FF_LongAcc_cpp", FF_LongAcc);
    logger_->log("FF_Aero_cpp", FF_Aero_N);
    logger_->log("FF_RollingResistance_cpp", FF_RollingResistance_N);
    logger_->log("a_x", feedback_ax_buffer_);
  }
  // Inputs
  void set_feedback_acceleration(
    const tam::types::control::AccelerationwithCovariances & accelerations)
  {
    feedback_ax_buffer_ = accelerations.acceleration_mps2.x;
  };
  void set_feedback_odometry(const tam::types::control::Odometry & odometry)
  {
    velocity_buffer_ = tam::helpers::geometry::euclidean_norm(odometry.velocity_mps);
  }
  void set_long_acc_target_mps2(const double & long_acc_target_mps2)
  {
    LongAcc_target_mps2_ = long_acc_target_mps2;
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
  // Outputs
  double get_Fx_command() const { return Fx_LongControlRequest_N; }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_; }
};
}  // namespace tam::control

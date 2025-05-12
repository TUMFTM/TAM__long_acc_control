#pragma once
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_helpers_cpp/constants.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
// region Controller Interface
namespace tam::control
{
class BrakePressureController
{
  using dpw_t = tam::types::common::DataPerWheel<double>;
  using bpw_t = tam::types::common::DataPerWheel<bool>;

private:
  // parameters
  struct
  {
    double tS;
    double FB_error_Ts;
    double FB_Kp;
    double FB_Ki;
    double FB_Kd;
    double FB_active;
    double brake_pressure_limit_bar;
    double saturation_pos_bar;
    double saturation_neg_bar;
    int smith_delay_comp_steps;
    std::vector<double> adaptive_gains;
    std::vector<double> adaptive_gain_breakpoints;
  } p_;
  // param manager
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash = 0;

  // debug container
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  // buffer
  dpw_t brake_pressure_target_bar_{};
  dpw_t brake_pressure_bar_{};
  std::vector<dpw_t> brake_pressure_target_buffer_{std::vector<dpw_t>(6, dpw_t(0.0))};
  bpw_t brake_pressure_saturation_triggered_{};
  dpw_t brake_pressure_request_bar_{};
  tam::helpers::control::PIDControl<dpw_t> pid_brake_pressure{};

  // operation
  bool EnableDriver_{}, EnableEmergency_{};
  void declare_and_update_parameters()
  {
    auto decl = [this](std::string name, double val) {
      return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    };
    p_.tS = decl("tS_LongitudinalController", 0.01);
    p_.FB_error_Ts = decl("P_VDC_brake_pressure_FB_error_Ts", 0.03);
    p_.FB_Kp = decl("P_VDC_brake_pressure_FB_Kp", 0.15);
    p_.FB_Ki = decl("P_VDC_brake_pressure_FB_Ki", 3.0);
    p_.FB_Kd = decl("P_VDC_brake_pressure_FB_Kd", 0.0);
    p_.FB_active = decl("P_VDC_brake_pressure_FB_active", 1.0);
    p_.brake_pressure_limit_bar = 1e-5 * decl("vehicle.brake.max_pressure", 12500000.0);
    p_.saturation_pos_bar = decl("P_VDC_brake_pressure_saturation_pos_bar", 10.0);
    p_.saturation_neg_bar = decl("P_VDC_brake_pressure_saturation_neg_bar", -2.0);

    p_.smith_delay_comp_steps =
      param_manager_
        ->declare_and_get_value(
          "P_VDC_brake_pressure_smith_delay_comp_steps", 6, tam::pmg::ParameterType::INTEGER, "")
        .as_int();

    p_.adaptive_gains =
      param_manager_
        ->declare_and_get_value(
          "brake_pressure_adaptive_gains", std::vector<double>{0.0, 0.3, 1.0, 2.0, 1.0, 0.3, 0.0},
          tam::pmg::ParameterType::DOUBLE_ARRAY, "")
        .as_double_array();

    p_.adaptive_gain_breakpoints = param_manager_
                                     ->declare_and_get_value(
                                       "brake_pressure_adaptive_gain_breakpoints",
                                       std::vector<double>{-10.0, -6.0, -1.0, 0.0, 1.0, 6.0, 10.0},
                                       tam::pmg::ParameterType::DOUBLE_ARRAY, "")
                                     .as_double_array();
    previous_param_state_hash = param_manager_->get_state_hash();

    // pid controller
    pid_brake_pressure.set_params(
      dpw_t(std::exp(-p_.tS / p_.FB_error_Ts)), dpw_t(p_.FB_Kp), dpw_t(p_.FB_Ki), dpw_t(p_.FB_Kd),
      dpw_t(p_.tS), dpw_t(p_.saturation_neg_bar), dpw_t(p_.saturation_pos_bar));

    // initialize buffer
    brake_pressure_target_buffer_.resize(p_.smith_delay_comp_steps, dpw_t(0.0));
  }

public:
  BrakePressureController()
  {
    // params
    declare_and_update_parameters();
  }
  // Non controller specific
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const { return param_manager_; }
  // Step method
  void step()
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash) {
      declare_and_update_parameters();
    }
    // Write target and actual values in vectors

    // initialize vectors
    dpw_t brake_pressure_error_bar;
    dpw_t brake_pressure_error_adaptive_bar;
    dpw_t brake_pressure_feedback_bar;
    tam::helpers::control::PIDFeedback<dpw_t> brake_pressure_fb;

    // update target buffer
    std::shift_left(brake_pressure_target_buffer_.begin(), brake_pressure_target_buffer_.end(), 1);
    brake_pressure_target_buffer_.back() = brake_pressure_target_bar_;
    // get brake pressure error
    brake_pressure_error_bar = brake_pressure_target_buffer_.front() - brake_pressure_bar_;
    dpw_t brake_pressure_error_gain;
    for (std::size_t i = 0; i < brake_pressure_error_gain.size(); i++) {
      brake_pressure_error_gain[i] = std::max(
        tam::helpers::numerical::interp(
          brake_pressure_error_bar[i], p_.adaptive_gain_breakpoints, p_.adaptive_gains),
        p_.adaptive_gains.front());
    }
    // adaptive error
    brake_pressure_error_adaptive_bar = brake_pressure_error_gain * brake_pressure_error_bar;

    // PID logic
    bpw_t integrator_switch =
      (!brake_pressure_saturation_triggered_ ||
       brake_pressure_fb.error_integrator * brake_pressure_error_adaptive_bar <= 0.0) &&
      brake_pressure_target_bar_ > 1e-10 && bpw_t(EnableDriver_);

    // PID step
    brake_pressure_fb = pid_brake_pressure.step(
      brake_pressure_error_adaptive_bar, bpw_t(EnableDriver_), bpw_t(true), integrator_switch,
      bpw_t(false), bpw_t(EnableDriver_));

    // Saturate feedback
    brake_pressure_feedback_bar = std::min(
      dpw_t(p_.saturation_pos_bar),
      std::max(dpw_t(p_.saturation_neg_bar), brake_pressure_fb.feedback));
    brake_pressure_saturation_triggered_ =
      std::abs(brake_pressure_feedback_bar - brake_pressure_fb.feedback) > 1e-10;
    brake_pressure_feedback_bar = brake_pressure_feedback_bar * p_.FB_active;

    // Add up brake pressure
    brake_pressure_request_bar_ =
      std::max(
        dpw_t(0.0), std::min(
                      dpw_t(p_.brake_pressure_limit_bar),
                      brake_pressure_target_bar_ + brake_pressure_feedback_bar)) *
      (brake_pressure_target_bar_ > 1e-10);
    // debug
    logger_->log("brake_pressure_FB_p_bar", brake_pressure_fb.feedback_p);
    logger_->log("brake_pressure_FB_i_bar", brake_pressure_fb.feedback_i);
    logger_->log("brake_pressure_FB_d_bar", brake_pressure_fb.feedback_d);
    logger_->log("brake_pressure_FB_bar", brake_pressure_feedback_bar);
    logger_->log("brake_pressure_target_bar", brake_pressure_target_bar_);
    logger_->log("brake_pressure_error_bar", brake_pressure_error_bar);
    logger_->log("brake_pressure_error_adaptive_bar", brake_pressure_error_adaptive_bar);
    logger_->log("brake_pressure_request_bar", brake_pressure_request_bar_);
  }
  // Inputs
  void set_target_brake_pressure_bar(
    const tam::types::common::DataPerWheel<double> & brake_pressure_target_bar)
  {
    brake_pressure_target_bar_ = brake_pressure_target_bar;
  }
  void set_feedback_brake_pressure_Pa(
    const tam::types::common::DataPerWheel<double> & brake_pressure_Pa)
  {
    brake_pressure_bar_ = brake_pressure_Pa / tam::constants::pascal_per_bar;
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
  tam::types::common::DataPerWheel<double> get_brake_pressure_command_Pa() const
  {
    return brake_pressure_request_bar_ * tam::constants::pascal_per_bar;
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_; }
};
}  // namespace tam::control
   // endregion
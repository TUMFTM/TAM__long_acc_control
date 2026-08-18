// Copyright 2024 Phillip Pitschi
#pragma once

#include <cmath>

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/param_manager_composer.hpp"
#include "tsl_logger_cpp/composer.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"

// Components
#include "long_acc_controller_tam_cpp/components/ax_controller.hpp"
#include "long_acc_controller_tam_cpp/components/brake_pressure_controller.hpp"
#include "long_acc_controller_tam_cpp/components/fx_controller.hpp"
namespace tam::control
{
class LongAccControllerCpp{
  using AxController_t = tam::control::AxController;
  using BrakePressureController_t = tam::control::BrakePressureController;
  using FxController_t = tam::control::FxController;

private:
  int8_t gear_request_{};
  // Components
  std::unique_ptr<AxController_t> ax_control{};
  std::unique_ptr<BrakePressureController_t> brake_pressure_control{};
  std::unique_ptr<FxController_t> fx_control{};

  tam::types::control::ICECommand ice_command_{};  // Output struct
  // Integration
  tam::tsl::LoggerComposer::SharedPtr logger_composer_ = std::make_shared<tam::tsl::LoggerComposer>(
    std::vector<tam::tsl::LoggerAccessInterface::SharedPtr>());
  // Setup the composing param manager
  tam::pmg::ParamManagerComposer::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamManagerComposer>(
      std::vector<tam::pmg::MgmtInterface::SharedPtr>{
        ax_control->get_param_handler(),
        brake_pressure_control->get_param_handler(), fx_control->get_param_handler()});

public:
  // Constructor and deconstructor
  LongAccControllerCpp();
  // Non controller specific
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() { return param_manager_; }
  // Step method
  void step();

  // Inputs
  void set_feedback_acceleration(
    const tam::types::control::AccelerationwithCovariances & accelerations);
  void set_feedback_drivetrain(
    const tam::types::control::DriveTrainFeedback & drivetrain_feedback);
  void set_target_brake_warmup_pressure_Pa(
    const tam::types::common::DataPerWheel<double> & brake_pressure_Pa);
  void set_feedback_brake_pressure_Pa(
    const tam::types::common::DataPerWheel<double> & brake_pressure_Pa);
  void set_feedback_odometry(const tam::types::control::Odometry & odometry);
  void set_long_acc_target_mps2(const double & long_acc_target_mps);
  void set_gear_control_request(const int8_t & gear_control_req);
  void set_slip_control_active(const bool status);
  // Outputs
  tam::types::control::ICECommand get_ice_commands() { return ice_command_; }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() { return logger_composer_; };
};
}  // namespace tam::control

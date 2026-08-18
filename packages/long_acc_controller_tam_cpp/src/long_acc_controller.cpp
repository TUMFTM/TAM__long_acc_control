// Copyright 2024 Simon Sagmeister
#include "long_acc_controller_tam_cpp/long_acc_controller.hpp"
namespace tam::control
{
  LongAccControllerCpp::LongAccControllerCpp()
  : ax_control(std::make_unique<AxController_t>()),
    brake_pressure_control(std::make_unique<BrakePressureController_t>()),
    fx_control(std::make_unique<FxController_t>())
  {
    // add module loggers to composer
    logger_composer_->register_logger(ax_control->get_debug_out(), "ax_control/");
    logger_composer_->register_logger(fx_control->get_debug_out(), "fx_control/");
    logger_composer_->register_logger(
      brake_pressure_control->get_debug_out(), "brake_pressure_control/");
  }
void LongAccControllerCpp::step()
{
  
  // Run slip calc and ax_control
  ax_control->step();

  // Set wheelslips to the corresponding modules
  fx_control->set_Fx_request_N(ax_control->get_Fx_command());
  fx_control->step();  // Step fx_control

  brake_pressure_control->set_target_brake_pressure_bar(
    fx_control->get_brake_pressure_target_bar());

  brake_pressure_control->step();

  // Set the output
  ice_command_.brake_pressure_Pa = brake_pressure_control->get_brake_pressure_command_Pa();
  ice_command_.gear = gear_request_;
  ice_command_.throttle = fx_control->get_throttle_command();
}
// Inputs
void LongAccControllerCpp::set_feedback_acceleration(
  const tam::types::control::AccelerationwithCovariances & accelerations)
{
  ax_control->set_feedback_acceleration(accelerations);
  fx_control->set_feedback_acceleration(accelerations);
}
void LongAccControllerCpp::set_feedback_drivetrain(
  const tam::types::control::DriveTrainFeedback & drivetrain_feedback)
{
  fx_control->set_feedback_drivetrain(drivetrain_feedback);
}
void LongAccControllerCpp::set_feedback_brake_pressure_Pa(
  const tam::types::common::DataPerWheel<double> & brake_pressure_Pa)
{
  brake_pressure_control->set_feedback_brake_pressure_Pa(brake_pressure_Pa);
}
void LongAccControllerCpp::set_target_brake_warmup_pressure_Pa(
  const tam::types::common::DataPerWheel<double> & brake_pressure_Pa)
{
  fx_control->set_target_brake_warmup_pressure_Pa(std::clamp(
    brake_pressure_Pa, tam::types::common::DataPerWheel<double>(0.0),
    tam::types::common::DataPerWheel<double>(15.0 * tam::constants::pascal_per_bar)));
}
void LongAccControllerCpp::set_feedback_odometry(const tam::types::control::Odometry & odometry)
{
  ax_control->set_feedback_odometry(odometry);
  fx_control->set_feedback_odometry(odometry);
}
void LongAccControllerCpp::set_long_acc_target_mps2(const double & long_acc_target_mps)
{
  fx_control->set_long_acc_target_mps2(long_acc_target_mps);
  ax_control->set_long_acc_target_mps2(long_acc_target_mps);
}
void LongAccControllerCpp::set_gear_control_request(const int8_t & gear_control_req)
{
  gear_request_ = gear_control_req;
  fx_control->set_gear_request(gear_request_);
}
void LongAccControllerCpp::set_slip_control_active(const bool status)
{
  fx_control->set_slip_control_active(status);
  ax_control->set_slip_control_active(status);
}
}  // namespace tam::control

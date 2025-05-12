// Copyright 2024 Simon Sagmeister
#include "long_acc_controller_tam_cpp/long_acc_controller.hpp"
namespace tam::control
{
  LongAccControllerCpp::LongAccControllerCpp()
  : slip_calculation(std::make_unique<SlipCalculation_t>()),
    ax_control(std::make_unique<AxController_t>()),
    brake_pressure_control(std::make_unique<BrakePressureController_t>()),
    fx_control(std::make_unique<FxController_t>()),
    slip_control(std::make_unique<SlipController_t>())
  {
    // add module loggers to composer
    logger_composer_->register_logger(slip_calculation->get_debug_out(), "slip_calculation/");
    logger_composer_->register_logger(ax_control->get_debug_out(), "ax_control/");
    logger_composer_->register_logger(fx_control->get_debug_out(), "fx_control/");
    logger_composer_->register_logger(slip_control->get_debug_out(), "slip_control/");
    logger_composer_->register_logger(
      brake_pressure_control->get_debug_out(), "brake_pressure_control/");
  }
void LongAccControllerCpp::step()
{
  // Set the inputs from the last cycle
  fx_control->set_slip_control_status(slip_control_status);
  ax_control->set_slip_control_status(slip_control_status);

  // Run slip calc and ax_control
  slip_calculation->step();
  ax_control->step();

  // Set wheelslips to the corresponding modules
  fx_control->set_gear_request(gear_request_);
  fx_control->set_Fx_request_N(ax_control->get_Fx_command());

  fx_control->step();  // Step fx_control

  // Set inputs of slip control
  slip_control->set_feedback_slips(
    slip_calculation->get_wheelslips(), slip_calculation->get_slip_angles());
  slip_control->set_feedback_slip_valid(true);
  slip_control->set_feedback_long_fx(ax_control->get_Fx_command());
  slip_control->set_feedback_target_brake_presure(fx_control->get_break_pressure_target());
  slip_control->set_feedback_throttle_request(fx_control->get_throttle_command());
  slip_control->set_feedback_gear(gear_request_);
  slip_control->step();  // Step slip_control

  brake_pressure_control->set_target_brake_pressure_bar(
    slip_control->get_break_pressure_target_bar());

  brake_pressure_control->step();

  // Set the output
  slip_control_status = slip_control->get_status();
  ice_command_.brake_pressure_Pa = brake_pressure_control->get_brake_pressure_command_Pa();
  ice_command_.gear = gear_request_;
  ice_command_.throttle = slip_control->get_throttle_request();
}
// Inputs
void LongAccControllerCpp::set_feedback_acceleration(
  const tam::types::control::AccelerationwithCovariances & accelerations)
{
  slip_calculation->set_feedback_acceleration(accelerations);
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
  slip_calculation->set_feedback_brake_pressure_Pa(brake_pressure_Pa);
  brake_pressure_control->set_feedback_brake_pressure_Pa(brake_pressure_Pa);
}
void LongAccControllerCpp::set_target_brake_warmup_pressure_Pa(
  const tam::types::common::DataPerWheel<double> & brake_pressure_Pa)
{
  fx_control->set_target_brake_warmup_pressure_Pa(std::clamp(
    brake_pressure_Pa, tam::types::common::DataPerWheel<double>(0.0),
    tam::types::common::DataPerWheel<double>(15.0 * tam::constants::pascal_per_bar)));
}
void LongAccControllerCpp::set_feedback_wheelspeed_radps(
  const tam::types::common::DataPerWheel<double> & wheelspeed_radps)
{
  slip_calculation->set_feedback_wheelspeed_radps(wheelspeed_radps);
}
void LongAccControllerCpp::set_feedback_odometry(const tam::types::control::Odometry & odometry)
{
  slip_calculation->set_feedback_odometry(odometry);
  ax_control->set_feedback_odometry(odometry);
  fx_control->set_feedback_odometry(odometry);
  slip_control->set_feedback_odometry(odometry);
}
void LongAccControllerCpp::set_feedback_steering_rad(
  const tam::types::control::AutowareSteeringReport & steering_report)
{
  slip_calculation->set_feedback_steering_rad(steering_report);
}
void LongAccControllerCpp::set_long_acc_target_mps2(const double & long_acc_target_mps)
{
  fx_control->set_long_acc_target_mps2(long_acc_target_mps);
  ax_control->set_long_acc_target_mps2(long_acc_target_mps);
}
void LongAccControllerCpp::set_operation_mode(
  const tam::types::control::AutowareOperationMode & operation_mode)
{
  slip_calculation->set_operation_mode(operation_mode);
  ax_control->set_operation_mode(operation_mode);
  fx_control->set_operation_mode(operation_mode);
  slip_control->set_operation_mode(operation_mode);
  brake_pressure_control->set_operation_mode(operation_mode);
}
void LongAccControllerCpp::set_gear_control_request(const int8_t & gear_control_req)
{
  gear_request_ = gear_control_req;
}
void LongAccControllerCpp::set_wheelspeed_ok(bool status)
{
  slip_calculation->set_wheelspeed_ok(status);
}
}  // namespace tam::control

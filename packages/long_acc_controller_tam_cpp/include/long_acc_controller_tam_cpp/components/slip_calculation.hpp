#pragma once
#include <algorithm>
#include <map>
#include <memory>

#include "controller_helpers_cpp/helpers.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include "vector"
// region Controller Interface
namespace tam::control
{
class SlipCalculation
{
  using dpw = tam::types::common::DataPerWheel<double>;

public:
  SlipCalculation() { declare_and_update_parameters(); }
  // Non controller specific
  tam::pmg::MgmtInterface::SharedPtr get_param_handler() const { return param_manager_; }
  // Step method

  void calc_dynamic_tire_radius()
  {
    // Interpolation function
    dynamic_tire_radius_m_ =
      dpw::from_front_and_rear(p_.tireradius_front_m_20mps, p_.tireradius_rear_m_20mps);
    if (p_.tire_radius_velocity_scaling_vel_points.empty()) {
      return;  // Disable by setting an empty vector
    }
    // Use state estimation velocity for calc here
    double vel = odometry_.velocity_mps.x;
    if (vel <= p_.tire_radius_velocity_scaling_vel_points[0]) {
      dynamic_tire_radius_m_ *= p_.tire_radius_velocity_scaling_scale_factors[0];
      return;
    }
    if (vel >= p_.tire_radius_velocity_scaling_vel_points.back()) {
      dynamic_tire_radius_m_ *= p_.tire_radius_velocity_scaling_scale_factors[0];
      return;
    }
    std::size_t first_bigger_element = 0;
    for (std::size_t i = 0; i < p_.tire_radius_velocity_scaling_vel_points.size(); i++) {
      if (p_.tire_radius_velocity_scaling_vel_points[i] > vel) {
        first_bigger_element = i;
        break;
      }
    }
    if (first_bigger_element <= 0) {
      return;
    }
    // Interpolate
    double scaling_factor =
      (vel - p_.tire_radius_velocity_scaling_vel_points[first_bigger_element - 1]) /
        (p_.tire_radius_velocity_scaling_vel_points[first_bigger_element] -
         p_.tire_radius_velocity_scaling_vel_points[first_bigger_element - 1]) *
        (p_.tire_radius_velocity_scaling_scale_factors[first_bigger_element] -
         p_.tire_radius_velocity_scaling_scale_factors[first_bigger_element - 1]) +
      p_.tire_radius_velocity_scaling_scale_factors[first_bigger_element - 1];

    dynamic_tire_radius_m_ *= scaling_factor;
  }
  void step()
  {
    if (param_manager_->get_state_hash() != previous_param_state_hash) {
      declare_and_update_parameters();
    }
    calc_dynamic_tire_radius();

    // Declare all local variables
    dpw vx_vehicle_mps, vy_vehicle_mps, vx_tire_mps, vy_tire_mps, v_tire_omega, sine, cose,
      alpha_rad, lambda, yaw_vx_vehicle_mps, vx_vehicle_mps_no_long_slip,
      vx_cog_vehicle_mps_no_long_slip_per_wheel;
    double low_speed_scale;

    // Calculate rotational velocity
    v_tire_omega = wheelspeed_radps_ * dynamic_tire_radius_m_;

    // Calc vy in the vehicle frame
    vy_vehicle_mps = dpw::from_front_and_rear(p_.l_front_m, -(p_.wheelbase_m - p_.l_front_m)) *
                       odometry_.angular_velocity_radps.z +
                     odometry_.velocity_mps.y;

    // Transformation from vehicle frame into the tire frame
    sine = dpw::from_front_and_rear(sin(-steering_report_.steering_angle_tire_rad), 0);
    cose = dpw::from_front_and_rear(cos(-steering_report_.steering_angle_tire_rad), 1);

    // Calculate yaw velocity in x direction (vehicle frame) for all tires
    yaw_vx_vehicle_mps = dpw::from_front_and_rear(p_.tw_front_m, p_.tw_rear_m) *
                         dpw::from_left_and_right(-1, 1) * 0.5 * odometry_.angular_velocity_radps.z;

    // region Calculate vx from the assumption that front wheel have 0 longitudinal slip
    vx_vehicle_mps_no_long_slip = (v_tire_omega + sine * vy_vehicle_mps) / cose;
    vx_cog_vehicle_mps_no_long_slip_per_wheel = vx_vehicle_mps_no_long_slip - yaw_vx_vehicle_mps;
    double vx_cog_vehicle_mps_no_long_slip =
      0.5 * (vx_cog_vehicle_mps_no_long_slip_per_wheel.front_left +
             vx_cog_vehicle_mps_no_long_slip_per_wheel.front_right);
    // endregion

    // region Blend between state estimation and front wheel based velocity
    double max_brake_pressure =
      std::max(brake_pressure_Pa_.front_left, brake_pressure_Pa_.front_right);
    double accel_current_long = pt1_filter_accel_long_.step(acceleration_.acceleration_mps2.x);
    double interp_factor = std::max(
      std::max(
        std::clamp(max_brake_pressure / 5e5, 0.0, 1.0),
        std::clamp(-(accel_current_long + 5.0), 0.0, 1.0)),
      std::clamp(
        std::max(std::abs(wheelslip_.front_left), std::abs(wheelslip_.front_right)) / 5.0 - 1.0,
        0.0, 1.0));
    double interp_factor_filtered = pt1_filter_speed_interp_.step(interp_factor);
    double vehicle_velocity_x_mps = interp_factor_filtered * odometry_.velocity_mps.x +
                                    (1 - interp_factor_filtered) * vx_cog_vehicle_mps_no_long_slip;
    // endregion

    // Calc velocity over ground for each wheel
    vx_vehicle_mps = yaw_vx_vehicle_mps + vehicle_velocity_x_mps;

    // Rotate front wheels to tire frame
    vx_tire_mps = cose * vx_vehicle_mps - sine * vy_vehicle_mps;
    vy_tire_mps = sine * vx_vehicle_mps + cose * vy_vehicle_mps;

    // Calc Slips
    constexpr double slip_fade = 2.0;
    low_speed_scale = std::clamp(
      std::pow(std::max(vehicle_velocity_x_mps - std::max((p_.v_min - slip_fade), 0.0), 0.0), 8) /
        std::pow(std::max(slip_fade, 0.1), 8),
      0.0, 1.0);
    lambda = ((v_tire_omega - vx_tire_mps) / std::max(vx_tire_mps, p_.v_min)) * low_speed_scale;
    alpha_rad = std::atan2(-vy_tire_mps, std::max(vx_tire_mps, p_.v_min)) * low_speed_scale;

    // Set output
    wheelslip_ = lambda * 100;
    slip_angle_ = alpha_rad;

    if (!wheelspeed_ok_) {
      wheelslip_ = dpw(0.0);
    }

    // Set debug outputs
    logger_->log("long_slip", wheelslip_);
    logger_->log("slip_angle_rad", alpha_rad);
    logger_->log("low_speed_scale", low_speed_scale);
    logger_->log("velocity_interpolation/factor", interp_factor);
    logger_->log("velocity_interpolation/factor_filtered", interp_factor_filtered);
    logger_->log(
      "velocity_interpolation/vx_cog_no_long_slip_front", vx_cog_vehicle_mps_no_long_slip);
    logger_->log("velocity_interpolation/vx_state_est", odometry_.velocity_mps.x);
    logger_->log(
      "tire_rolling_radius/at_20mps",
      dpw::from_front_and_rear(p_.tireradius_front_m_20mps, p_.tireradius_rear_m_20mps));
    logger_->log("tire_rolling_radius/dynamic", dynamic_tire_radius_m_);
  }
  // Inputs
  void set_feedback_wheelspeed_radps(
    const tam::types::common::DataPerWheel<double> & wheelspeed_radps)
  {
    wheelspeed_radps_ = wheelspeed_radps;
  }
  void set_wheelspeed_ok(bool status) { wheelspeed_ok_ = status; }
  void set_feedback_odometry(const tam::types::control::Odometry & odometry)
  {
    odometry_ = odometry;
  }
  void set_feedback_acceleration(
    const tam::types::control::AccelerationwithCovariances & acceleration)
  {
    acceleration_ = acceleration;
  }
  void set_feedback_brake_pressure_Pa(
    const tam::types::common::DataPerWheel<double> & brake_pressure_Pa)
  {
    brake_pressure_Pa_ = brake_pressure_Pa;
  }
  void set_operation_mode(const tam::types::control::AutowareOperationMode & operation_mode)
  {
    operation_mode_ = operation_mode;
  }
  void set_feedback_steering_rad(
    const tam::types::control::AutowareSteeringReport & steering_report)
  {
    steering_report_ = steering_report;
  }
  // Outputs
  tam::types::common::DataPerWheel<double> get_wheelslips() const { return wheelslip_; }
  tam::types::common::DataPerWheel<double> get_slip_angles() const { return slip_angle_; }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_; }

private:
  // Inputs
  bool wheelspeed_ok_{};
  tam::types::common::DataPerWheel<double> wheelspeed_radps_{};
  tam::types::control::Odometry odometry_{};
  tam::types::control::AccelerationwithCovariances acceleration_{};
  tam::types::common::DataPerWheel<double> brake_pressure_Pa_{};
  tam::types::control::AutowareOperationMode operation_mode_{};
  tam::types::control::AutowareSteeringReport steering_report_{};

  // Outputs
  tam::types::common::DataPerWheel<double> wheelslip_;
  tam::types::common::DataPerWheel<double> slip_angle_;

  // Helpers
  tam::types::common::DataPerWheel<double> dynamic_tire_radius_m_;
  tam::helpers::control::FirstOrderLowPass<double> pt1_filter_accel_long_ = {0.0, 0.8};
  tam::helpers::control::FirstOrderLowPass<double> pt1_filter_speed_interp_ = {1.0, 0.8};
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash = 0;
  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();
  void declare_and_update_parameters()
  {
    p_.v_min = param_manager_
                 ->declare_and_get_value(
                   "P_VDC_MinVelSlipCalc_mps", 3.0, tam::pmg::ParameterType::DOUBLE, "")
                 .as_double();
    p_.tw_front_m =
      param_manager_
        ->declare_and_get_value(
          "vehicle.dimension.track_width_front", 1.639, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    p_.tw_rear_m =
      param_manager_
        ->declare_and_get_value(
          "vehicle.dimension.track_width_rear", 1.524, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    p_.l_front_m =
      param_manager_
        ->declare_and_get_value(
          "vehicle.dimension.distance_to_front_axle", 1.724, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    p_.wheelbase_m = param_manager_
                       ->declare_and_get_value(
                         "vehicle.dimension.wheelbase", 2.971, tam::pmg::ParameterType::DOUBLE, "")
                       .as_double();
    p_.tireradius_front_m_20mps =
      param_manager_
        ->declare_and_get_value(
          "tires.front_left.radius.radius_20mps", 0.293475, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    p_.tireradius_rear_m_20mps =
      param_manager_
        ->declare_and_get_value(
          "tires.rear_left.radius.radius_20mps", 0.3074348, tam::pmg::ParameterType::DOUBLE, "")
        .as_double();
    p_.tire_radius_velocity_scaling_vel_points =
      param_manager_
        ->declare_and_get_value(
          "tires.front_left.velocity_scaling.velocity", std::vector<double>({20, 50, 80}),
          tam::pmg::ParameterType::DOUBLE_ARRAY,
          "Velocity points for the corresponding scaling of the tire rolling diameter")
        .as_double_array();
    p_.tire_radius_velocity_scaling_scale_factors =
      param_manager_
        ->declare_and_get_value(
          "tires.front_left.velocity_scaling.factor", std::vector<double>({1.0, 1.0065, 1.0205}),
          tam::pmg::ParameterType::DOUBLE_ARRAY,
          "Scaling factor for the velocity induced increase in tire rolling diameter")
        .as_double_array();
    previous_param_state_hash = param_manager_->get_state_hash();
  }
  // Parameters
  struct
  {
    double v_min;
    double tw_front_m;
    double tw_rear_m;
    double l_front_m;
    double wheelbase_m;
    double tireradius_front_m_20mps;
    double tireradius_rear_m_20mps;
    std::vector<double> tire_radius_velocity_scaling_vel_points{};
    std::vector<double> tire_radius_velocity_scaling_scale_factors{};
  } p_;
};
}  // namespace tam::control
// endregion

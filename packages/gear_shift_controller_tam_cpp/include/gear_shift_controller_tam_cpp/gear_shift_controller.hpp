#pragma once

#include <param_management_ros2_integration_cpp/helper_functions.hpp>
#include <rclcpp/rclcpp.hpp>

#include "controller_helpers_cpp/helpers.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "param_management_cpp/param_value_manager.hpp"
#include "tier4_planning_msgs/msg/trajectory.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tum_helpers_cpp/coordinate_system/curvilinear_cosy.hpp"
#include "tum_msgs/msg/tum_additional_info_point.hpp"
#include "tum_msgs/msg/tum_additional_trajectory_infos.hpp"
#include "tum_msgs/msg/tum_control_constraints.hpp"
#include "tum_msgs/msg/tum_float32_stamped.hpp"
#include "tum_msgs/msg/tum_float64_per_wheel_stamped.hpp"
#include "tum_msgs/msg/tum_int8_stamped.hpp"
#include "tum_msgs/msg/tum_longitudinal_cmd.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
#include "tum_types_cpp/common.hpp"
#include "tum_types_cpp/control.hpp"
#include <vector>
#include <memory>
namespace tam::control
{
class GearShiftController
{
private:
  struct
  {
    double n_ub_rpm;
    double r_tire_rear;
    double n_ub_max_acc_rpm;
    double shift_safety_factor;
    double c_diff_ratio;
    double shift_safety_factor_predictive;
    double shift_wait_time_ms;
    std::size_t lookahead_traj_pts;
    double eng_downshift_reduction_rpm;
    double eng_limiter_safety_revs_rpm;
    double retry_failed_shift_ms;
    double ay_filter_pole;
    std::vector<double> i_gearset_table;
    std::vector<double> gear_omega_engine_min_table;
  } p_;
  tam::pmg::ParamValueManager::SharedPtr param_manager_ =
    std::make_shared<tam::pmg::ParamValueManager>();
  std::size_t previous_param_state_hash = 0;

  tam::tsl::ValueLogger::SharedPtr logger_ = std::make_shared<tam::tsl::ValueLogger>();

  void declare_and_update_parameters();

  // Member Variables
  // Gear buffer
  uint8_t gear_command_{};
  uint8_t current_gear_{1};
  int8_t current_gear_request_{-1};
  double current_i_{0.0};
  std::chrono::steady_clock::time_point time_last_gearshift_request_{
    std::chrono::steady_clock::now()};
  bool upshift_flag_{false};
  bool downshift_flag_{false};
  bool hold_gear_flag_{false};
  double radps_lb_{0.0};
  double radps_ub_{0.0};
  tam::types::control::ControlConstraints constr_buffer_{};
  // Sensor Inputs
  double omega_eng_radps_{0.0};
  double filtered_ay_{0.0};

  tam::helpers::control::FirstOrderLowPass<double> ay_filter{0.0, 0.7};

  tam::types::control::Trajectory traj_buffer_{};
  tam::helpers::cosy::CurvilinearCosyPtr cosy_{};

  tam::types::control::Odometry odometry_buffer_{};

public:
  GearShiftController(/* args */);
  void set_feedback_omega_engine(float omega_eng_radps);
  void set_feedback_gear(int8_t gear);
  void set_feedback_trajectory(
    tam::types::control::Trajectory traj, tam::types::control::ControlConstraints constr);
  void set_feedback_odometry(tam::types::control::Odometry odometry);
  void set_feedback_acceleration(double ay);
  void step();
  bool get_upshift_flag() { return upshift_flag_; }
  uint8_t get_gear_command() { return gear_command_; }
  bool get_downshift_flag() { return downshift_flag_; }
  tam::pmg::ParamValueManager::SharedPtr get_param_manager() const { return param_manager_; }
  tam::tsl::LoggerAccessInterface::SharedPtr get_debug_out() const { return logger_; }
};
}  // namespace tam::control

#include "gear_shift_controller_tam_cpp/gear_shift_controller.hpp"

#include <cmath>
namespace tam::control
{
GearShiftController::GearShiftController() { declare_and_update_parameters(); }
void GearShiftController::step()
{
  if (param_manager_->get_state_hash() != previous_param_state_hash) {
    declare_and_update_parameters();
  }
  radps_lb_ = p_.gear_omega_engine_min_table[current_gear_] * ((2.0 * M_PI) / 60.0);
  radps_ub_ = p_.n_ub_rpm * ((2.0 * M_PI) / 60.0);

  double ay_max_safety_current{0.0};
  double current_ax_target{-1};  // Init with -1 to not block downshifts without having a trajectory
  double current_v_target{0};

  auto clamp_index = [this](std::size_t index) {
    return std::clamp(index, 0ul, traj_buffer_.points.size() - 1);
  };

  double ay_max_safety{0.0};
  double calc_omega_radps{0.0};

  if (cosy_) {
    std::size_t next_traj_pt_index = std::clamp<float>(
      std::ceil(std::get<1>(cosy_->convert_to_sn_and_get_idx(odometry_buffer_))), 0,
      traj_buffer_.points.size() -
        1);  // First clamp and then convert to size_t otherwise it will undefined behavior

    current_ax_target = traj_buffer_.points[next_traj_pt_index].acceleration_mps2.x;
    current_v_target = traj_buffer_.points[next_traj_pt_index].velocity_mps.x;

    ay_max_safety_current = std::min(
                              std::abs(constr_buffer_.points[next_traj_pt_index].a_y_max_mps2.y),
                              std::abs(constr_buffer_.points[next_traj_pt_index].a_y_min_mps2.y)) *
                            p_.shift_safety_factor;

    /*
     * for next 4 seconds:
     * check vx * current gear ratio --> shift required? --> yes? --> shift_req_flag = Ture
     * check ay in step before --> shift still possible? --> shift_req_flag = False
     * else: check previous step --> repeat
     * if current time step or next 0.5 s --> calc current rpm --> if okay --> shift request
     */
    for (std::size_t i = traj_buffer_.points.size() - 1;
         i >= clamp_index(next_traj_pt_index + p_.lookahead_traj_pts);
         i--)  // check every 0.08 seconds
    {
      ay_max_safety = std::min(
                        std::abs(constr_buffer_.points[i].a_y_max_mps2.y),
                        std::abs(constr_buffer_.points[i].a_y_min_mps2.y)) *
                      p_.shift_safety_factor_predictive;
      calc_omega_radps = current_gear_ > 0
                           ? p_.i_gearset_table[current_gear_ - 1] * p_.c_diff_ratio *
                               std::abs(traj_buffer_.points[i].velocity_mps.x) / p_.r_tire_rear
                           : 0.0;
      if (calc_omega_radps >= radps_ub_) {
        upshift_flag_ = true;  // shift required
      }
      if (calc_omega_radps <= radps_lb_) {
        downshift_flag_ = true;  // shift required
      }
      // shift still possible in a later point
      if (std::abs(traj_buffer_.points[i].acceleration_mps2.y) <= ay_max_safety) {
        upshift_flag_ = false;
        if (
          calc_omega_radps >= radps_lb_ + p_.eng_downshift_reduction_rpm * ((2.0 * M_PI) / 60.0)) {
          downshift_flag_ = false;
        }
      }
      if (
        traj_buffer_.points[clamp_index(next_traj_pt_index + 3)].velocity_mps.x <=
        odometry_buffer_.velocity_mps.x) {  // Should this be a deceleration check????
        hold_gear_flag_ = true;
      }
    }
  }

  double radps_cut = p_.n_ub_max_acc_rpm * ((2.0 * M_PI) / 60.0);

  bool shift_req = false;
  bool cut_shift_req = false;
  // initialize
  if (omega_eng_radps_ > 50 && current_gear_request_ == -1) {
    current_gear_request_ = current_gear_;
    time_last_gearshift_request_ = std::chrono::steady_clock::now();
  }
  // only start shifting after it has been initialized
  if (current_gear_request_ != -1) {
    // prevent gear jumping by waiting some time until gear shift is tried again
    if (
      std::chrono::duration_cast<std::chrono::milliseconds>(
        (std::chrono::steady_clock::now() - time_last_gearshift_request_))
        .count() >= p_.shift_wait_time_ms) {
      // upshift
      if (
        ((omega_eng_radps_ >= radps_ub_)) &&
        current_gear_ < static_cast<int8_t>(p_.i_gearset_table.size())) {
        shift_req = true;
        if (std::abs(filtered_ay_) > ay_max_safety_current) {
          if (omega_eng_radps_ >= radps_cut) {
            current_gear_request_ = current_gear_ + 1;
          } else {
            cut_shift_req = true;
          }
        } else {
          current_gear_request_ = current_gear_ + 1;
          time_last_gearshift_request_ = std::chrono::steady_clock::now();
        }

        // downshift
      } else if (((omega_eng_radps_ <= radps_lb_) || downshift_flag_) && current_gear_ > 1) {
        if (
          (omega_eng_radps_ * p_.i_gearset_table[current_gear_ - 2] /
           p_.i_gearset_table[current_gear_ - 1]) <=
          (radps_ub_ - (p_.eng_limiter_safety_revs_rpm) * ((2.0 * M_PI) / 60.0))) {
          shift_req = true;
          if (
            std::abs(filtered_ay_) > ay_max_safety_current ||
            (current_ax_target > 0.0 && omega_eng_radps_ > 2000 * ((2.0 * M_PI) / 60.0) &&
             current_v_target > 5.0)) {
            // Cut downshift in the corner + during acceleration (to save the poor gearbox)
            cut_shift_req = true;
          } else {
            current_gear_request_ = current_gear_ - 1;
            time_last_gearshift_request_ = std::chrono::steady_clock::now();
          }
        }
      }
    } else {
      // A2RL: 0.4 s
      // 0.6 seconds until shift is required to be completed % if not completed, reset current
      // request and try again after full second has expired
      if (
        (std::chrono::duration_cast<std::chrono::milliseconds>(
           (std::chrono::steady_clock::now() - time_last_gearshift_request_))
           .count() >= p_.retry_failed_shift_ms) &&
        current_gear_request_ != current_gear_) {
        current_gear_request_ = current_gear_;
      }
    }
    gear_command_ = current_gear_request_;
  } else {
    gear_command_ = current_gear_;
  }
  logger_->log("calc_omega_radps_predictive", calc_omega_radps);
  logger_->log("ay_max_safety_predictive", ay_max_safety);
  logger_->log("radps_cut", radps_cut);
  logger_->log("radps_lb", radps_lb_);
  logger_->log("radps_ub", radps_ub_);
  logger_->log("filtered_ay", filtered_ay_);
  logger_->log("calc_ay_max_safety", ay_max_safety_current);
  logger_->log("downshift_flag", downshift_flag_);
  logger_->log("upshift_flag", upshift_flag_);
  logger_->log("hold_gear_flag", hold_gear_flag_);
  logger_->log("shift_request", shift_req);
  logger_->log("cut_shift_req", cut_shift_req);
  logger_->log("current_gear_request", current_gear_request_);
  logger_->log("gear_command", gear_command_);

  upshift_flag_ = false;
  downshift_flag_ = false;
  hold_gear_flag_ = false;
}
void GearShiftController::declare_and_update_parameters()
{
  auto decl = [this](std::string name, double val) {
    return param_manager_->declare_and_get_value(name, val, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  };
  p_.i_gearset_table = param_manager_
                         ->declare_and_get_value(
                           "vehicle.drivetrain.gear_ratios",
                           std::vector<double>({0.0, 2.9167, 1.875, 1.3809, 1.1154, 0.96, 0.8889}),
                           tam::pmg::ParameterType::DOUBLE_ARRAY, "")
                         .as_double_array();
  p_.gear_omega_engine_min_table =
    param_manager_
      ->declare_and_get_value(
        "GSC_gear_omega_engine_min_table",
        std::vector<double>({2200.0, 2200.0, 2200.0, 3000.0, 3400.0, 3600.0, 3700.0}),
        tam::pmg::ParameterType::DOUBLE_ARRAY, "")
      .as_double_array();
  p_.n_ub_rpm = decl("GSC_P_VDC_POWTR__ENGINE__n_ub_rpm", 4800.0);
  p_.r_tire_rear = decl("tires.rear_left.radius.radius_20mps", 0.3088);
  p_.n_ub_max_acc_rpm = decl("GSC_P_VDC_POWTR__ENGINE__n_ub_max_acc_rpm", 8000.0);
  p_.shift_safety_factor = decl("GSC_P_VDC_shift_safety_factor", 0.6);
  p_.c_diff_ratio = decl("vehicle.drivetrain.transmission_ratio", 2.818);
  p_.shift_safety_factor_predictive = decl("GSC_P_VDC_shift_safety_factor_predictive", 0.3);
  p_.shift_wait_time_ms = decl("GSC_shift_wait_time_ms", 420.0);
  p_.lookahead_traj_pts =
    param_manager_
      ->declare_and_get_value("GSC_lookahead_traj_pts", 3, tam::pmg::ParameterType::INTEGER, "")
      .as_int();
  p_.eng_downshift_reduction_rpm = decl("GSC_eng_downshift_reduction_rpm", -600.0);
  p_.eng_limiter_safety_revs_rpm = decl("GSC_eng_limiter_safety_revs_rpm", 450.0);
  p_.retry_failed_shift_ms = decl("GSC_retry_failed_shift_ms", 450.0);
  p_.ay_filter_pole = decl("GSC_ay_filter_pole", 0.7);
  previous_param_state_hash = param_manager_->get_state_hash();

  // filter
  ay_filter.set_tf_pole(p_.ay_filter_pole);
}
void GearShiftController::set_feedback_omega_engine(float omega_eng_radps)
{
  omega_eng_radps_ = omega_eng_radps;
}
void GearShiftController::set_feedback_gear(int8_t gear) { current_gear_ = gear; }
void GearShiftController::set_feedback_trajectory(
  tam::types::control::Trajectory traj, tam::types::control::ControlConstraints constr)
{
  cosy_ = tam::helpers::cosy::CurvilinearCosy::create(traj)->build();
  traj_buffer_ = traj;
  constr_buffer_ = constr;
}
void GearShiftController::set_feedback_odometry(tam::types::control::Odometry odometry)
{
  odometry_buffer_ = odometry;
}
void GearShiftController::set_feedback_acceleration(double ay)
{
  filtered_ay_ = ay_filter.step(ay);
}
};  // namespace tam::control

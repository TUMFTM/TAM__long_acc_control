#pragma once
#include <numeric>
#include <vector>

#include "controller_helpers_cpp/helpers.hpp"
namespace tam::control
{
struct SlipControlStatus
{
  bool abs_latched = false;
  bool tc_latched = false;
};
enum class Slip_Control_Type { Abs = 0, Tc = 1 };
enum class Wheel_Position { Front_Left = 0, Front_Right = 1, Rear_Left = 2, Rear_Right = 3 };
enum class Wheel_States {
  Not_Changed = -1,
  Not_Latched = 0,
  Reduction_Phase = 1,
  Increase_Phase = 2,
  Hold_Phase = 3,
};
enum class OperationMode { individual, front_rear_split, all_together, rear_only };
struct AbsTcInputs
{
  double long_fx{};
  double slip{};
  bool slip_valid{};
  double brake_pressure{};
  bool allowed{};
  tam::types::control::Odometry odometry{};
  double slip_angle{};
  double slip_lookahead{};
  double slip_rate{};
};
struct SlipControlInputs
{
  double long_fx{};
  tam::types::common::DataPerWheel<double> slip_input_abs{};
  tam::types::common::DataPerWheel<double> slip_input_tc{};
  tam::types::common::DataPerWheel<double> slip_angle_abs{};
  tam::types::common::DataPerWheel<double> slip_angle_tc{};
  tam::types::common::DataPerWheel<double> slip_lookahead_abs{};
  tam::types::common::DataPerWheel<double> slip_lookahead_tc{};
  tam::types::common::DataPerWheel<double> slip_rate_abs{};
  bool slip_valid{};
  tam::types::common::DataPerWheel<double> target_brake_pressure{};
  tam::types::control::Odometry odometry{};
  double throttle_request{};
  int8_t gear{};
};
struct SlipControlState
{
  tam::types::common::DataPerWheel<double> brake_pressure_target_bar{};
  double throttle_target{};
  std::chrono::steady_clock::time_point time_gear_changed{};
  std::chrono::steady_clock::time_point time_slip_control_latched{};
  double activation_distance{};
  bool abs_latched{};
  bool tc_latched{};
  tam::types::common::DataPerWheel<double> slip{};
  tam::types::common::DataPerWheel<double> slip_rate{};
  tam::types::common::DataPerWheel<double> slip_rate_filtered{};
  std::chrono::time_point<std::chrono::steady_clock> last_call_time{};
};
struct SlipControlParams
{
  bool abs_is_activated{};
  OperationMode abs_operation_mode{};
  double abs_min_activation_velocity{};
  double abs_min_activation_acceleration{};
  double abs_cooldown_gear_change{};
  double tc_cooldown_gear_change{};
  bool tc_is_activated{};
  OperationMode tc_operation_mode{};
  double tc_min_activation_velocity{};
  double P_VDC_MaxSlipThrottleCut{};
  double slip_angle_filter_pole{};
  double slip_rate_filter_pole{};
  double tS{};
  double lookahead_horizon{};
};
struct AbsState
{
  Wheel_States wheel_state{Wheel_States::Not_Latched};
  double eps{};
  double reduction_factor{};
  tam::helpers::control::Cyclic_Vector<double> long_fx_input_vector;
  double long_fx_input_average{};
  double long_fx_latched{};
  double long_fx_reference{};
  double long_fx_output{};
  tam::helpers::control::Cyclic_Vector<double> long_fx_output_vector{};
  double long_fx_output_average{};
  double brake_pressure_latched{};
  double brake_pressure_output{};
  double v_latched{};
  bool safety_feature_is_active{false};
  double slip_target_reduction{};
  double slip_error{};
};
struct TcState
{
  Wheel_States wheel_state{Wheel_States::Not_Latched};
  double current_slip_{};
  bool tc_allowed_{false};
  double min_ax_;
  double min_eps;
  double eps;
  double target_adjustment_factor_{};
  double target_fx_trc_{};
  double long_fx_input_latched{};
  double long_fx_output{};
  double target_brake_pressure_latched{};
  double brake_pressure_target_bar_{};
  double target_brake_pressure_output{};
  double slip_target_reduction{};
  double slip_error{};
  tam::types::control::Odometry odom_{};
  std::chrono::time_point<std::chrono::steady_clock> last_time_threshold_exceeded{};
};
}  // namespace tam::control
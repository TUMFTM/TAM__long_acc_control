#pragma once
#include <algorithm>
#include <initializer_list>
#include <iostream>
#include <variant>
#include <vector>

#include "tum_helpers_cpp/numerical.hpp"
#include "tum_types_cpp/control.hpp"
namespace tam::helpers::control
{
tam::types::control::ControlConstraintPoint find_constraint_point(
  const tam::types::control::ControlConstraints & constraints, const float idx);
tam::types::control::TrajectoryPoint find_trajectory_point(
  const tam::types::control::Trajectory & trajectory, const float idx);
tam::types::control::AdditionalInfoPoint find_additional_info_point(
  const tam::types::control::AdditionalTrajectoryInfos & additional_info, const float idx);
double find_heading(const tam::types::control::Trajectory & trajectory, const float idx);
template <typename TrajectoryType, typename TrajectoryPointType>
TrajectoryType extract_trajectory_starting_at_idx(
  const TrajectoryType & in, const int idx, const int length)
{
  TrajectoryType out{};
  out.header = in.header;
  if (std::max(idx, 0) + length < in.points.size()) {
    std::copy(
      in.points.begin() + std::max(idx, 0), in.points.begin() + std::max(idx, 0) + length,
      std::back_inserter(out.points));
  } else {
    if (in.points.size() >= length) {
      std::copy(in.points.end() - length, in.points.end(), std::back_inserter(out.points));
    } else {
      std::cout << "[TrackingController]: Trajectory has less than N_Planner = " << length
                << "values!";
    }
  }
  while (out.points.size() < length) {
    TrajectoryPointType point = out.points[-1];
    out.points.push_back(point);
  }
  return out;
}
template <typename T>
class FirstOrderLowPass
{
private:
  T pole_;
  T old_output_;

public:
  FirstOrderLowPass() = default;
  FirstOrderLowPass(const T & initial_output, const T & pole_value);
  T step(const T & input);
  void set_tf_pole(const T & tf_pole);
  void set_old_output(const T & old_output);
};
template <typename T>
struct PIDFeedback
{
  T feedback;
  T feedback_p;
  T feedback_i;
  T feedback_d;
  T error_integrator;
};
template <typename T>
class PIDControl
{
public:
  PIDControl() = default;
  PIDControl(
    const T & tf_pole, const T & kp, const T & ki, const T & kd, const T & tS,
    const T & saturation_low, const T & saturation_high);
  template <typename U>
  PIDFeedback<T> step(
    const T & error, const U & update_pd, const U & use_pd, const U & update_i, const U & reset_i,
    const U & use_i)
  {
    // update transfer function with new error
    T tf_error = d_filter_.step(error * update_pd);

    PIDFeedback<T> fb;
    // P-Feedback
    fb.feedback_p = kp_ * tf_error * use_pd;
    // D-Feedback
    fb.feedback_d = kd_ * (tf_error - error_old_) / tS_ * use_pd;
    // update error buffer
    error_old_ = tf_error;

    // update integrator
    error_integrator_ *= !reset_i;  // Reset integrator
    T integrator_update = error_integrator_ + (ki_ * error * tS_) * update_i;

    error_integrator_ +=
      (integrator_update - error_integrator_) *
      (integrator_update < saturation_high_ && integrator_update > saturation_low_);
    fb.error_integrator = error_integrator_;
    // I-feedback
    fb.feedback_i = error_integrator_ * use_i;

    // add up feedback
    fb.feedback = fb.feedback_p + fb.feedback_i + fb.feedback_d;

    return fb;
  }
  template <typename U>
  PIDFeedback<T> step(
    const T & error_pd, const T & error_i, const U & update_pd, const U & use_pd,
    const U & update_i, const U & reset_i, const U & use_i)
  {
    // update transfer function with new error
    T tf_error = d_filter_.step(error_pd * update_pd);

    PIDFeedback<T> fb;
    // P-Feedback
    fb.feedback_p = kp_ * tf_error * use_pd;
    // D-Feedback
    fb.feedback_d = kd_ * (tf_error - error_old_) / tS_ * use_pd;
    // update error buffer
    error_old_ = tf_error;

    // update integrator
    error_integrator_ *= !reset_i;  // Reset integrator
    T integrator_update = error_integrator_ + (ki_ * error_i * tS_) * update_i;

    error_integrator_ +=
      (integrator_update - error_integrator_) *
      (integrator_update < saturation_high_ && integrator_update > saturation_low_);
    fb.error_integrator = error_integrator_;
    // I-feedback
    fb.feedback_i = error_integrator_ * use_i;

    // add up feedback
    fb.feedback = fb.feedback_p + fb.feedback_i + fb.feedback_d;

    return fb;
  }
  void set_params(
    const T & tf_pole, const T & kp, const T & ki, const T & kd, const T & tS,
    const T & saturation_low, const T & saturation_high);

private:
  T kp_{}, ki_{}, kd_{}, tS_{}, error_integrator_{}, error_old_{}, saturation_low_{},
    saturation_high_{};
  FirstOrderLowPass<T> d_filter_;
};
template <typename T>
class Cyclic_Vector
{
private:
  std::vector<T> data;
  unsigned int index = 1;

public:
  Cyclic_Vector() = default;
  Cyclic_Vector(int size) : data(std::vector<T>(size)) {}
  Cyclic_Vector(std::initializer_list<T> initializer_list)
  {
    for (auto & elem : initializer_list) {
      data.push_back(elem);
    }
  }
  Cyclic_Vector(int size, T initial_data)
  {
    data = std::vector<T>(size);
    std::fill(data.begin(), data.end(), initial_data);
  }
  void insert(T new_element)
  {
    index++;
    if (index == data.size()) index = 0;
    data.at(index) = new_element;
  }
  std::vector<T>::iterator begin() { return data.begin(); }
  std::vector<T>::iterator end() { return data.end(); }
  int size() { return data.size(); }
  std::vector<T> get_data() { return data; }
  void resize(int count)
  {
    if (count < data.size()) {
      index = count - 1;
    }
    if (count != data.size()) {
      data.resize(count);
    }
  }
  int get_index() { return index; }
  // Position is relative to index
  T get_element(int position) { return data.at((index + data.size() + position) % data.size()); }
};
template <typename T>
class Flank_Detector
{
private:
  T data{};
  double eps{1e-5};

public:
  Flank_Detector() = default;
  explicit Flank_Detector(double eps_) : eps(eps_) {}
  bool check_and_update(T new_data)
  {
    if (data != new_data) {
      data = new_data;
      return true;
    }
    return false;
  }
  bool check_and_update(double new_data)
  {
    if (abs(data - new_data) > eps) {
      data = new_data;
      return true;
    }
    return false;
  }
};
}  // namespace tam::helpers::control
#include "controller_helpers_cpp/helpers_impl.hpp"

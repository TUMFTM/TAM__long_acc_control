#pragma once
#include "controller_helpers_cpp/helpers.hpp"
namespace tam::helpers::control
{
template <typename T>
FirstOrderLowPass<T>::FirstOrderLowPass(const T & initial_output, const T & pole_value)
: pole_(pole_value), old_output_(initial_output)
{
}
template <typename T>
T FirstOrderLowPass<T>::step(const T & input)
{
  T new_output = pole_ * old_output_ + (-pole_ + 1.0) * input;
  old_output_ = new_output;
  return new_output;
}
template <typename T>
void FirstOrderLowPass<T>::set_tf_pole(const T & tf_pole)
{
  pole_ = tf_pole;
}
template <typename T>
void FirstOrderLowPass<T>::set_old_output(const T & old_output)
{
  old_output_ = old_output;
}
template <typename T>
PIDControl<T>::PIDControl(
  const T & tf_pole, const T & kp, const T & ki, const T & kd, const T & tS,
  const T & saturation_low, const T & saturation_high)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  tS_(tS),
  saturation_low_(saturation_low),
  saturation_high_(saturation_high),
  d_filter_(FirstOrderLowPass(T(), tf_pole))
{
}
template <typename T>
void PIDControl<T>::set_params(
  const T & tf_pole, const T & kp, const T & ki, const T & kd, const T & tS,
  const T & saturation_low, const T & saturation_high)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  tS_ = tS;
  saturation_low_ = saturation_low;
  saturation_high_ = saturation_high;
  d_filter_.set_tf_pole(tf_pole);
}
}  // namespace tam::helpers::control
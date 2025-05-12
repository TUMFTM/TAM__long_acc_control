// Copyright 2023 Simon Sagmeister
#include "controller_helpers_cpp/engine_map.hpp"
namespace tam::control::engine
{
EngineMap::EngineMap(tam::pmg::ParamValueManager::SharedPtr pmg) : pmg_(pmg)
{
  declare_and_update_parameters();
}
void EngineMap::declare_and_update_parameters()
{
  this->rpm_min =
    pmg_->declare_and_get_value("EngineMap.rpm_min", 0.0, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  this->rpm_max =
    pmg_->declare_and_get_value("EngineMap.rpm_max", 0.0, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  this->rpm_num_values =
    pmg_->declare_and_get_value("EngineMap.rpm_num_values", 0, tam::pmg::ParameterType::INTEGER, "")
      .as_int();
  this->throttle_min =
    pmg_->declare_and_get_value("EngineMap.throttle_min", 0.0, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  this->throttle_max =
    pmg_->declare_and_get_value("EngineMap.throttle_max", 0.0, tam::pmg::ParameterType::DOUBLE, "")
      .as_double();
  this->throttle_num_values =
    pmg_
      ->declare_and_get_value("EngineMap.throttle_num_values", 0, tam::pmg::ParameterType::INTEGER, "")
      .as_int();

  auto engine_map_parameter_data = pmg_
                                     ->declare_and_get_value(
                                       "EngineMap.engine_map", std::vector<double>{},
                                       tam::pmg::ParameterType::DOUBLE_ARRAY, "")
                                     .as_double_array();

  if (engine_map_parameter_data.size() != rpm_num_values * throttle_num_values) {
    throw std::runtime_error("Engine map data size does not match the given dimensions");
  }

  // Initialize engine map memory
  this->engine_map_data =
    std::vector<std::vector<double>>(rpm_num_values, std::vector<double>(throttle_num_values));

  std::size_t index = 0;
  for (std::size_t i = 0; i < rpm_num_values; ++i) {
    for (std::size_t j = 0; j < throttle_num_values; ++j) {
      this->engine_map_data[i][j] = engine_map_parameter_data[index++];
    }
  }
}
double EngineMap::lookup_engine_map_Nm(const double & omega_engine_rad, const double & throttle)
{
  double n_engine_rpm = 60 * omega_engine_rad / (2 * M_PI);
  if (n_engine_rpm > this->rpm_max) return -100;  // rpm limiter
  if (n_engine_rpm < this->rpm_min) return 0;     // engine can't produce torque
  std::size_t rpm_index = std::round(
    (std::clamp(n_engine_rpm, this->rpm_min, this->rpm_max) - this->rpm_min) /
    (this->rpm_max - this->rpm_min) * (this->rpm_num_values - 1));
  std::size_t throttle_index = std::round(
    (std::clamp(throttle, this->throttle_min, this->throttle_max) - this->throttle_min) /
    (this->throttle_max - this->throttle_min) * (this->throttle_num_values - 1));
  return this->engine_map_data.at(rpm_index).at(throttle_index);
}
double EngineMap::lookup_throttle_value(const double & omega_engine_rad, const double & T_engine_Nm)
{
  double n_engine_rpm = 60 * omega_engine_rad / (2 * M_PI);
  if (n_engine_rpm > this->rpm_max) return 0.0;  // rpm limiter
  if (n_engine_rpm < this->rpm_min) return 0.0;  // engine can't produce torque
  std::size_t rpm_index = std::round(
    (std::clamp(n_engine_rpm, this->rpm_min, this->rpm_max) - this->rpm_min) /
    (this->rpm_max - this->rpm_min) * (this->rpm_num_values - 1));
  std::size_t throttle_index = 0;
  if (this->engine_map_data.at(rpm_index).at(this->throttle_num_values - 1) < T_engine_Nm)
    return 1.0;
  for (std::size_t i = 0; i < this->throttle_num_values; i++) {
    if (this->engine_map_data.at(rpm_index).at(i) > T_engine_Nm) {
      throttle_index = i;
      break;
    }
  }
  return std::clamp(
    this->throttle_min +
      throttle_index * (this->throttle_max - this->throttle_min) / (this->throttle_num_values - 1),
    0.0, 1.0);
}
}  // namespace tam::control::engine

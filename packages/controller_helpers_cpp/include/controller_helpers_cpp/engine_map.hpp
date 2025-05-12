// Copyright 2023 Simon Sagmeister

#pragma once
#include <math.h>

#include <algorithm>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <string>
#include <unordered_map>

#include "param_management_cpp/param_value_manager.hpp"
namespace tam::control::engine
{
class EngineMap
{
public:
  explicit EngineMap(tam::pmg::ParamValueManager::SharedPtr pmg);
  void declare_and_update_parameters();
  double lookup_engine_map_Nm(const double & omega_engine_rad, const double & throttle);
  double lookup_throttle_value(const double & omega_engine_rad, const double & T_engine_Nm);

private:
tam::pmg::ParamValueManager::SharedPtr pmg_;
  std::vector<std::vector<double>> engine_map_data{};
  double rpm_min{};
  double rpm_max{};
  size_t rpm_num_values{};
  double throttle_min{};
  double throttle_max{};
  size_t throttle_num_values{};
};
}  // namespace tam::control::engine

// Copyright 2023 Tobias Betz
#include <rclcpp/rclcpp.hpp>

#include "brake_temperature_controller_node_cpp/brake_temperature_controller_node.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<tam::control::BrakeTemperatureControllerNode>(options);
  tam::pmg::validate_param_overrides(argc, argv, node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// Copyright 2025 Phillip Pitschi
#include <rclcpp/rclcpp.hpp>

#include "gear_shift_controller_tam_cpp/gear_shift_controller.hpp"
#include "gear_shift_controller_node_cpp/gear_shift_controller_node.hpp"
#include "param_management_ros2_integration_cpp/helper_functions.hpp"
#include "vehicle_handler_cpp/vehicle_handler.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::unique_ptr<tam::common::VehicleHandler> vehicle_ =
    tam::common::VehicleHandler::from_pkg_config();
  vehicle_->init_param_backend();

  rclcpp::NodeOptions options;
  auto node = std::make_shared<tam::control::GearShiftControllerNode>(std::make_unique<tam::control::GearShiftController>(), options);
  tam::pmg::validate_param_overrides(argc, argv, node.get());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

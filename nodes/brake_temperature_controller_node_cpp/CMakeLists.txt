# Copyright 2023 Simon Sagmeister
cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================

set(PROJECT_NAME brake_temperature_controller_node_cpp)
set(NODE_EXECUTABLE_NAME brake_temperature_controller_node)
set(NODE_LIBRARY_PATH src/brake_temperature_controller_node.cpp)
set(NODE_EXECUTABLE_PATH src/brake_temperature_controller_node_main.cpp)

set(NODE_DEPENDENCIES
  rclcpp
  autoware_auto_control_msgs
  tum_msgs
  param_management_ros2_integration_cpp
  ros2_watchdog_cpp
  tum_types_cpp
  tum_helpers_cpp
  geometry_msgs
  controller_helpers_cpp
  tier4_planning_msgs
  tsl_logger_cpp
  tsl_ros2_publisher_cpp
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD LIBRARY TARGET TO IMPORT NODE CLASS WITHIN OTHER PKGS
# ====================================================================

add_library(${PROJECT_NAME} SHARED ${NODE_LIBRARY_PATH})

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_target_dependencies(${PROJECT_NAME} ${NODE_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)
ament_export_dependencies(${NODE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

# section: BUILD EXECUTABLE
# ====================================================================
add_executable(${NODE_EXECUTABLE_NAME} ${NODE_EXECUTABLE_PATH})

target_link_libraries(${NODE_EXECUTABLE_NAME} ${PROJECT_NAME})

install(
  TARGETS ${NODE_EXECUTABLE_NAME}  
  DESTINATION lib/${PROJECT_NAME}
)
# section: GENERAL
# ====================================================================
ament_package()
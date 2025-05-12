cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME long_acc_controller_tam_cpp)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(param_management_cpp REQUIRED)
find_package(tum_types_cpp REQUIRED)
find_package(tum_helpers_cpp REQUIRED)
find_package(controller_helpers_cpp REQUIRED)
find_package(tsl_logger_cpp REQUIRED)

#add_subdirectory(src)

## Build wrapper library
add_library(${PROJECT_NAME} SHARED 
  src/long_acc_controller.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -fopenmp) #-dndebug -fno-math-errno -fopenmp)

target_include_directories(${PROJECT_NAME} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} 
  param_management_cpp
  tum_types_cpp
  controller_helpers_cpp
  tum_helpers_cpp
  tsl_logger_cpp
)

# Install library
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME} # Here always project name.. above library name
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Export ament targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(tod_core::tod_core)
ament_export_dependencies(param_management_cpp
                          tum_types_cpp
                          controller_helpers_cpp
                          tum_helpers_cpp
                          tsl_logger_cpp) # other dependencies so client does not need to call find_package on them
ament_export_include_directories(include) # superfluous: handeled by INCLUDES DESTINATION

ament_package()

// Copyright 2023 TUMWFTM
#pragma once
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "ros2_watchdog_cpp/node_monitor.hpp"
#include "tum_type_conversions_ros_cpp/tum_type_conversions.hpp"
class LongitudinalControllerStateMachine
{
public:
  enum longitudinal_controller_state_ { startup = 20, operation = 30, timeout = 50 };
  LongitudinalControllerStateMachine();
  enum Signals {
    Acceleration,
    Odometry,
    Omega_Engine,
    Gear,
    Brake_Pressure,
    Target_Acceleration,
    Gear_Request,
    Slip_Control_Active,
    CNT_NUM_SIGNALS
  };
  struct DiagnosticReturn
  {
    longitudinal_controller_state_ state;
    tam::types::ErrorLvl error_lvl;
    std::string message;
    const std::string causing_key = "state_machine";
    std::array<bool, Signals::CNT_NUM_SIGNALS> timeout_detected;
  };
  DiagnosticReturn get_diagnostic_state();
  longitudinal_controller_state_ GetLongitudinalControllerState();
  bool CheckIsTimeouted(LongitudinalControllerStateMachine::Signals signal);
  void SetReceivedOnce(Signals signal);
  std::function<void(bool, std::chrono::milliseconds)> GetTimeoutFunction(Signals signal);

private:
  void SetTimeoutedTopics(bool timeout, std::chrono::milliseconds timeout_now, Signals signal);
  void UpdateStateMachine();

  std::vector<std::tuple<Signals, tam::types::ErrorLvl, std::string>> signal_info = {
    {Signals::Acceleration, tam::types::ErrorLvl::ERROR, "acc"},
    {Signals::Odometry, tam::types::ErrorLvl::ERROR, "odo"},
    {Signals::Omega_Engine, tam::types::ErrorLvl::ERROR, "w_eng"},
    {Signals::Gear, tam::types::ErrorLvl::ERROR, "gear"},
    {Signals::Brake_Pressure, tam::types::ErrorLvl::ERROR, "p_br"},
    {Signals::Target_Acceleration, tam::types::ErrorLvl::ERROR, "tar_acc"},
    {Signals::Gear_Request, tam::types::ErrorLvl::ERROR, "gear_req"},
    {Signals::Slip_Control_Active, tam::types::ErrorLvl::WARN, "slip_ctrl"}};

  std::array<bool, Signals::CNT_NUM_SIGNALS> received_once_;
  std::array<bool, Signals::CNT_NUM_SIGNALS> timeout_detected_;

  longitudinal_controller_state_ state_;
};

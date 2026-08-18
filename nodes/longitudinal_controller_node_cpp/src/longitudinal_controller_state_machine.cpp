// Copyright 2023 Simon Sagmeister
#include "longitudinal_controller_node_cpp/longitudinal_controller_state_machine.hpp"
LongitudinalControllerStateMachine::LongitudinalControllerStateMachine()
{
  received_once_.fill(false);
  timeout_detected_.fill(false);
  state_ = longitudinal_controller_state_::startup;
  received_once_[Signals::Slip_Control_Active] = true;  // Slip control is not always used, so we don't want to wait for it
}
LongitudinalControllerStateMachine::DiagnosticReturn
LongitudinalControllerStateMachine::get_diagnostic_state()
{
  UpdateStateMachine();
  DiagnosticReturn diag_return;
  diag_return.timeout_detected = timeout_detected_;
  diag_return.state = state_;

  switch (state_) {
    case longitudinal_controller_state_::startup:
      diag_return.error_lvl = tam::types::ErrorLvl::ERROR;
      diag_return.message = "Wait:";
      for (auto & [signal, _, name] : signal_info) {
        if (received_once_[signal]) continue;
        diag_return.message += name + "|";
      }
      diag_return.message.pop_back();
      if (diag_return.message.size() > 20) diag_return.message.erase(20);
      break;
    case longitudinal_controller_state_::operation:
      diag_return.error_lvl = tam::types::ErrorLvl::OK;
      diag_return.message = "Ok";
      break;
    case longitudinal_controller_state_::timeout:
      diag_return.error_lvl = tam::types::ErrorLvl::OK;
      diag_return.message = "T_out:";
      for (auto & [signal, assigned_severity, name] : signal_info) {
        if (!timeout_detected_[signal]) continue;
        diag_return.message += name + "|";
        diag_return.error_lvl = std::max(diag_return.error_lvl, assigned_severity);
      }
      diag_return.message.pop_back();
      if (diag_return.message.size() > 20) diag_return.message.erase(20);
      break;
  }
  return diag_return;
}
LongitudinalControllerStateMachine::longitudinal_controller_state_
LongitudinalControllerStateMachine::GetLongitudinalControllerState()
{
  return state_;
}
bool LongitudinalControllerStateMachine::CheckIsTimeouted(
  LongitudinalControllerStateMachine::Signals signal)
{
  if (timeout_detected_[signal]) {
    return true;
  }
  return false;
}
void LongitudinalControllerStateMachine::UpdateStateMachine()
{
  switch (state_) {
    case longitudinal_controller_state_::startup:
      if (std::all_of(received_once_.begin(), received_once_.end(), [](bool i) { return i; })) {
        state_ = longitudinal_controller_state_::operation;
      }
      break;

    case longitudinal_controller_state_::operation:
      if (std::any_of(
            timeout_detected_.begin(), timeout_detected_.end(), [](bool i) { return i; })) {
        state_ = longitudinal_controller_state_::timeout;
      }
      break;
    case longitudinal_controller_state_::timeout:
      if (std::all_of(
            timeout_detected_.begin(), timeout_detected_.end(), [](bool i) { return !i; })) {
        state_ = longitudinal_controller_state_::operation;
      }
      break;
  }
}
void LongitudinalControllerStateMachine::SetReceivedOnce(
  LongitudinalControllerStateMachine::Signals signal)
{
  received_once_[signal] = true;
}
void LongitudinalControllerStateMachine::SetTimeoutedTopics(
  bool timeout, std::chrono::milliseconds, LongitudinalControllerStateMachine::Signals signal)
{
  timeout_detected_[signal] = timeout;
}
std::function<void(bool, std::chrono::milliseconds)>
LongitudinalControllerStateMachine::GetTimeoutFunction(
  LongitudinalControllerStateMachine::Signals signal)
{
  return std::bind(
    &LongitudinalControllerStateMachine::SetTimeoutedTopics, this, std::placeholders::_1,
    std::placeholders::_2, signal);
}

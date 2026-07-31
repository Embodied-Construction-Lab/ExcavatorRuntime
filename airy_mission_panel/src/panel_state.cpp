#include "airy_mission_panel/panel_state.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace airy_mission_panel
{

namespace
{
const std::array<JointTestSpec, kJointTestCount> kJointTestSpecs{{
  {"swing_joint", -157, 157, 0},
  {"boom_joint", -314, 314, 0},
  {"arm_joint", -314, 314, 0},
  {"bucket_joint", -314, 314, 0},
}};
}  // namespace

PanelView derive_panel_view(
  const RuntimeSnapshot & runtime,
  const OperatorResources & resources,
  OwnedOperation owned_operation)
{
  PanelView view;
  const bool safe_shadow =
    runtime.received && runtime.fresh &&
    runtime.execution_mode == "shadow" && runtime.motion_backend == "none" &&
    !runtime.motion_authorized && !runtime.sender_constructed &&
    runtime.quiescent && runtime.action_datagrams == 0 &&
    runtime.active_behavior.empty();
  const bool safe_control =
    runtime.received && runtime.fresh && runtime.input_source == "live" &&
    runtime.execution_mode == "control" && runtime.motion_backend == "orin_edge" &&
    runtime.control_stage == "commissioning" &&
    runtime.motion_authorized && runtime.sender_constructed && runtime.quiescent &&
    runtime.state_fresh && runtime.control_enabled && runtime.sensor_valid &&
    runtime.stm32_alive && !runtime.estop && runtime.fault_free &&
    runtime.motion_gate_reason == "ready" &&
    runtime.active_behavior.empty();
  const bool orin_edge_follow =
    runtime.motion_backend == "orin_edge" && runtime.follow_control_mode == "edge_onnx";
  const bool safe_follow_control = safe_control && orin_edge_follow;
  const bool safe_fixed_control = safe_control;
  const bool safe_full_mission_control = safe_control;
  const bool idle = owned_operation == OwnedOperation::kIdle;

  view.plan_follow_dig_enabled =
    (safe_shadow || safe_follow_control) && idle && resources.dig_target_available;
  view.plan_follow_dump_enabled =
    (safe_shadow || safe_follow_control) && idle && resources.dump_target_available;
  view.return_home_enabled =
    safe_shadow && idle && resources.home_pose_available;
  view.execute_dig_enabled =
    safe_fixed_control && idle && resources.execute_dig_available;
  view.execute_dump_enabled =
    safe_fixed_control && idle && resources.execute_dump_available;
  view.full_mission_enabled =
    safe_full_mission_control && idle && resources.full_mission_available;
  if (orin_edge_follow) {
    view.follow_status_text = "ORIN EDGE FOLLOW / ONNX 100% / UNTIL RESULT OR CANCEL";
  } else if (safe_shadow) {
    view.follow_status_text = "SHADOW / NO MOTION";
  } else {
    view.follow_status_text = "FOLLOW LOCKED";
  }
  view.cancel_enabled = !idle;

  if (safe_shadow) {
    auto source = runtime.input_source;
    std::transform(
      source.begin(), source.end(), source.begin(),
      [](unsigned char character) {return std::toupper(character);});
    view.safety_text = source + " / SHADOW / READY";
  } else if (safe_control) {
    auto stage = runtime.control_stage;
    std::transform(
      stage.begin(), stage.end(), stage.begin(),
      [](unsigned char character) {return std::toupper(character);});
    view.safety_text = "LIVE / " + stage + " / READY";
  } else if (!runtime.received || !runtime.fresh) {
    view.safety_text = "LOCKED / RUNTIME STATUS UNAVAILABLE";
  } else if (
    runtime.motion_gate_reason == "behavior_active" &&
    !runtime.active_behavior.empty())
  {
    auto behavior = runtime.active_behavior;
    std::transform(
      behavior.begin(), behavior.end(), behavior.begin(),
      [](unsigned char character) {return std::toupper(character);});
    view.safety_text = "LIVE / BUSY / " + behavior;
  } else {
    auto reason = runtime.motion_gate_reason;
    std::transform(
      reason.begin(), reason.end(), reason.begin(),
      [](unsigned char character) {return std::toupper(character);});
    view.safety_text = reason.empty() ?
      "LOCKED / SAFETY CONTRACT NOT SATISFIED" : "LOCKED / " + reason;
  }
  return view;
}

std::vector<OperatorLogEntry> append_operator_log(
  const std::vector<OperatorLogEntry> & entries,
  std::uint8_t ros_level,
  std::int64_t stamp_ns,
  const std::string & module,
  const std::string & message,
  std::size_t max_entries)
{
  auto updated = entries;
  if (max_entries == 0) {
    return {};
  }
  const auto remove_oldest_warning = [&updated]() {
    const auto warning = std::find_if(
      updated.begin(), updated.end(), [](const OperatorLogEntry & entry) {
        return entry.severity == OperatorLogSeverity::kWarning;
      });
    if (warning != updated.end()) {
      updated.erase(warning);
      return true;
    }
    return false;
  };
  while (updated.size() > max_entries) {
    if (!remove_oldest_warning()) {
      updated.erase(updated.begin());
    }
  }
  if (ros_level < 30) {
    return updated;
  }

  OperatorLogSeverity severity = OperatorLogSeverity::kWarning;
  if (ros_level >= 50) {
    severity = OperatorLogSeverity::kFatal;
  } else if (ros_level >= 40) {
    severity = OperatorLogSeverity::kError;
  }
  if (!updated.empty()) {
    auto & latest = updated.back();
    if (latest.severity == severity && latest.module == module && latest.message == message) {
      latest.stamp_ns = stamp_ns;
      ++latest.repeat_count;
      return updated;
    }
  }
  if (updated.size() == max_entries) {
    if (!remove_oldest_warning()) {
      if (severity == OperatorLogSeverity::kWarning) {
        return updated;
      }
      updated.erase(updated.begin());
    }
  }
  updated.push_back(
    OperatorLogEntry{
      severity,
      stamp_ns,
      module.empty() ? "<unknown>" : module,
      message,
      1});
  return updated;
}

const std::array<JointTestSpec, kJointTestCount> & joint_test_specs()
{
  return kJointTestSpecs;
}

JointTestSample make_joint_test_sample(
  const std::array<int, kJointTestCount> & slider_ticks)
{
  JointTestSample sample;
  for (std::size_t index = 0; index < kJointTestCount; ++index) {
    const auto & spec = kJointTestSpecs[index];
    if (slider_ticks[index] < spec.lower_tick || slider_ticks[index] > spec.upper_tick) {
      throw std::out_of_range(spec.name + " test slider value is outside its configured range");
    }
    sample.names[index] = spec.name;
    sample.positions_rad[index] = static_cast<double>(slider_ticks[index]) * 0.01;
  }
  return sample;
}

bool joint_test_publishing_allowed(
  const RuntimeSnapshot & runtime,
  bool explicitly_enabled,
  std::size_t joint_state_publisher_count)
{
  return explicitly_enabled && joint_state_publisher_count <= 1 &&
         runtime.received && runtime.fresh && runtime.input_source == "fixture" &&
         runtime.execution_mode == "shadow" && runtime.motion_backend == "none" &&
         !runtime.motion_authorized && !runtime.sender_constructed &&
         runtime.action_datagrams == 0;
}

}  // namespace airy_mission_panel

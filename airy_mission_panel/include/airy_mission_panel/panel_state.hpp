#ifndef AIRY_MISSION_PANEL__PANEL_STATE_HPP_
#define AIRY_MISSION_PANEL__PANEL_STATE_HPP_

#include <array>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace airy_mission_panel
{

constexpr std::size_t kJointTestCount = 4;

struct JointTestSpec
{
  std::string name;
  int lower_tick{0};
  int upper_tick{0};
  int default_tick{0};
};

struct JointTestSample
{
  std::array<std::string, kJointTestCount> names;
  std::array<double, kJointTestCount> positions_rad;
};

enum class OwnedOperation
{
  kIdle,
  kPlanFollow,
  kExecuteDig,
  kExecuteDump,
  kFullMission,
  kReturnHome,
};

enum class OperatorLogSeverity
{
  kWarning,
  kError,
  kFatal,
};

struct OperatorLogEntry
{
  OperatorLogSeverity severity{OperatorLogSeverity::kWarning};
  std::int64_t stamp_ns{0};
  std::string module;
  std::string message;
  std::uint64_t repeat_count{1};
};

struct RuntimeSnapshot
{
  bool received{false};
  bool fresh{false};
  std::string input_source;
  std::string execution_mode;
  std::string control_stage;
  std::string motion_backend;
  bool motion_authorized{false};
  bool sender_constructed{false};
  bool quiescent{false};
  std::uint64_t action_datagrams{0};
  std::string active_behavior;
  bool state_fresh{false};
  bool control_enabled{false};
  bool sensor_valid{false};
  bool stm32_alive{false};
  bool estop{false};
  bool fault_free{false};
  bool fixed_actions_validated{false};
  std::string follow_control_mode;
  std::string motion_gate_reason;
  std::string last_rejection_reason;
  std::string last_rejection_message;
};

struct OperatorResources
{
  bool dig_target_available{false};
  bool dump_target_available{false};
  bool home_pose_available{false};
  bool execute_dig_available{false};
  bool execute_dump_available{false};
  bool full_mission_available{false};
};

struct PanelView
{
  bool plan_follow_dig_enabled{false};
  bool plan_follow_dump_enabled{false};
  bool return_home_enabled{false};
  bool cancel_enabled{false};
  bool execute_dig_enabled{false};
  bool execute_dump_enabled{false};
  bool full_mission_enabled{false};
  std::string safety_text;
  std::string follow_status_text;
};

PanelView derive_panel_view(
  const RuntimeSnapshot & runtime,
  const OperatorResources & resources,
  OwnedOperation owned_operation);

std::vector<OperatorLogEntry> append_operator_log(
  const std::vector<OperatorLogEntry> & entries,
  std::uint8_t ros_level,
  std::int64_t stamp_ns,
  const std::string & module,
  const std::string & message,
  std::size_t max_entries);

const std::array<JointTestSpec, kJointTestCount> & joint_test_specs();

JointTestSample make_joint_test_sample(
  const std::array<int, kJointTestCount> & slider_ticks);

bool joint_test_publishing_allowed(
  const RuntimeSnapshot & runtime,
  bool explicitly_enabled,
  std::size_t joint_state_publisher_count);

}  // namespace airy_mission_panel

#endif  // AIRY_MISSION_PANEL__PANEL_STATE_HPP_

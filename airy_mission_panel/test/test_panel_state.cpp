#include <gtest/gtest.h>

#include <array>

#include "airy_mission_panel/panel_state.hpp"

namespace airy_mission_panel
{
namespace
{

RuntimeSnapshot safe_runtime()
{
  RuntimeSnapshot runtime;
  runtime.received = true;
  runtime.fresh = true;
  runtime.input_source = "fixture";
  runtime.execution_mode = "shadow";
  runtime.motion_backend = "none";
  runtime.motion_authorized = false;
  runtime.sender_constructed = false;
  runtime.quiescent = true;
  runtime.action_datagrams = 0;
  return runtime;
}

RuntimeSnapshot safe_edge_control_runtime()
{
  RuntimeSnapshot runtime;
  runtime.received = true;
  runtime.fresh = true;
  runtime.input_source = "live";
  runtime.execution_mode = "control";
  runtime.control_stage = "commissioning";
  runtime.motion_backend = "orin_edge";
  runtime.motion_authorized = true;
  runtime.sender_constructed = true;
  runtime.quiescent = true;
  runtime.state_fresh = true;
  runtime.control_enabled = true;
  runtime.sensor_valid = true;
  runtime.stm32_alive = true;
  runtime.fault_free = true;
  runtime.motion_gate_reason = "ready";
  runtime.fixed_actions_validated = true;
  runtime.follow_control_mode = "edge_onnx";
  return runtime;
}

TEST(PanelState, EnablesOnlyImplementedActionsWithSafeResources)
{
  OperatorResources resources;
  resources.dig_target_available = true;
  resources.dump_target_available = true;
  resources.execute_dig_available = true;
  resources.execute_dump_available = true;
  resources.full_mission_available = true;
  resources.home_pose_available = true;

  const auto view = derive_panel_view(
    safe_runtime(), resources, OwnedOperation::kIdle);

  EXPECT_TRUE(view.plan_follow_dig_enabled);
  EXPECT_TRUE(view.plan_follow_dump_enabled);
  EXPECT_FALSE(view.execute_dig_enabled);
  EXPECT_FALSE(view.execute_dump_enabled);
  EXPECT_FALSE(view.full_mission_enabled);
  EXPECT_TRUE(view.return_home_enabled);
  EXPECT_FALSE(view.cancel_enabled);
  EXPECT_EQ(view.safety_text, "FIXTURE / SHADOW / READY");
}

TEST(PanelState, EnablesFinalLiveActionsOnlyWhenOrinEdgeIsReady)
{
  OperatorResources resources;
  resources.dig_target_available = true;
  resources.dump_target_available = true;
  resources.execute_dig_available = true;
  resources.execute_dump_available = true;
  resources.full_mission_available = true;

  auto runtime = safe_edge_control_runtime();
  runtime.fixed_actions_validated = false;
  auto view = derive_panel_view(runtime, resources, OwnedOperation::kIdle);
  EXPECT_TRUE(view.plan_follow_dig_enabled);
  EXPECT_TRUE(view.plan_follow_dump_enabled);
  EXPECT_TRUE(view.execute_dig_enabled);
  EXPECT_TRUE(view.execute_dump_enabled);
  EXPECT_TRUE(view.full_mission_enabled);
  EXPECT_EQ(view.safety_text, "LIVE / COMMISSIONING / READY");
  EXPECT_EQ(
    view.follow_status_text,
    "ORIN EDGE FOLLOW / ONNX 100% / UNTIL RESULT OR CANCEL");

  runtime.control_enabled = false;
  view = derive_panel_view(runtime, resources, OwnedOperation::kIdle);
  EXPECT_FALSE(view.plan_follow_dig_enabled);
  EXPECT_FALSE(view.execute_dig_enabled);

  runtime = safe_edge_control_runtime();
  runtime.motion_gate_reason = "mission_targets_not_field_validated";
  view = derive_panel_view(runtime, resources, OwnedOperation::kIdle);
  EXPECT_FALSE(view.plan_follow_dig_enabled);
  EXPECT_FALSE(view.execute_dig_enabled);
  EXPECT_EQ(view.safety_text, "LOCKED / MISSION_TARGETS_NOT_FIELD_VALIDATED");
}

TEST(PanelState, RejectsTheRetiredPcUdpPolicyBackend)
{
  OperatorResources resources;
  resources.dig_target_available = true;
  resources.dump_target_available = true;
  resources.execute_dig_available = true;
  resources.execute_dump_available = true;
  resources.full_mission_available = true;

  auto runtime = safe_edge_control_runtime();
  runtime.motion_backend = "udp_policy";
  const auto view = derive_panel_view(runtime, resources, OwnedOperation::kIdle);

  EXPECT_FALSE(view.plan_follow_dig_enabled);
  EXPECT_FALSE(view.plan_follow_dump_enabled);
  EXPECT_FALSE(view.execute_dig_enabled);
  EXPECT_FALSE(view.execute_dump_enabled);
  EXPECT_FALSE(view.full_mission_enabled);
  EXPECT_EQ(view.safety_text, "LOCKED / READY");
}

TEST(PanelState, ShowsActiveEdgeFollowAsBusyInsteadOfLockedReady)
{
  OperatorResources resources;
  resources.dig_target_available = true;
  resources.dump_target_available = true;
  auto runtime = safe_edge_control_runtime();
  runtime.quiescent = false;
  runtime.active_behavior = "Follow";
  runtime.motion_gate_reason = "behavior_active";

  const auto view = derive_panel_view(
    runtime, resources, OwnedOperation::kPlanFollow);

  EXPECT_EQ(view.safety_text, "LIVE / BUSY / FOLLOW");
  EXPECT_TRUE(view.cancel_enabled);
  EXPECT_FALSE(view.plan_follow_dig_enabled);
  EXPECT_FALSE(view.plan_follow_dump_enabled);
}

TEST(PanelState, FailsClosedWithoutFreshSafeRuntime)
{
  OperatorResources resources{true, true, true};
  auto runtime = safe_runtime();
  runtime.fresh = false;

  const auto view = derive_panel_view(
    runtime, resources, OwnedOperation::kIdle);

  EXPECT_FALSE(view.plan_follow_dig_enabled);
  EXPECT_FALSE(view.plan_follow_dump_enabled);
  EXPECT_FALSE(view.return_home_enabled);
  EXPECT_FALSE(view.cancel_enabled);
}

TEST(PanelState, ExposesOnlyCancelWhilePanelOwnsAnOperation)
{
  OperatorResources resources{true, true, true};

  const auto view = derive_panel_view(
    safe_runtime(), resources, OwnedOperation::kPlanFollow);

  EXPECT_FALSE(view.plan_follow_dig_enabled);
  EXPECT_FALSE(view.plan_follow_dump_enabled);
  EXPECT_FALSE(view.return_home_enabled);
  EXPECT_TRUE(view.cancel_enabled);
}

TEST(PanelState, RetainsOnlyBoundedWarningAndErrorLogsWithSource)
{
  std::vector<OperatorLogEntry> entries;

  entries = append_operator_log(entries, 20, 1, "fixture_plan_server", "ready", 2);
  entries = append_operator_log(entries, 30, 2, "bucket_tip_pose_publisher", "waiting for TF", 2);
  entries = append_operator_log(entries, 40, 3, "fixture_plan_server", "planning failed", 2);
  entries = append_operator_log(entries, 50, 4, "mission_behavior_server", "fatal state", 2);

  ASSERT_EQ(entries.size(), 2U);
  EXPECT_EQ(entries[0].severity, OperatorLogSeverity::kError);
  EXPECT_EQ(entries[0].stamp_ns, 3);
  EXPECT_EQ(entries[0].module, "fixture_plan_server");
  EXPECT_EQ(entries[0].message, "planning failed");
  EXPECT_EQ(entries[1].severity, OperatorLogSeverity::kFatal);
  EXPECT_EQ(entries[1].module, "mission_behavior_server");
}

TEST(PanelState, WarningFloodDoesNotEvictAnError)
{
  std::vector<OperatorLogEntry> entries;
  entries = append_operator_log(entries, 40, 1, "planner", "failed", 2);
  entries = append_operator_log(entries, 30, 2, "sensor", "late", 2);
  entries = append_operator_log(entries, 30, 3, "sensor", "late again", 2);

  ASSERT_EQ(entries.size(), 2U);
  EXPECT_EQ(entries[0].severity, OperatorLogSeverity::kError);
  EXPECT_EQ(entries[0].message, "failed");
  EXPECT_EQ(entries[1].message, "late again");
}

TEST(PanelState, CoalescesConsecutiveIdenticalLogsAndKeepsLatestStamp)
{
  std::vector<OperatorLogEntry> entries;
  entries = append_operator_log(
    entries, 30, 1, "octomap_server", "Nothing to publish, octree is empty", 100);
  entries = append_operator_log(
    entries, 30, 2, "octomap_server", "Nothing to publish, octree is empty", 100);
  entries = append_operator_log(
    entries, 30, 3, "octomap_server", "Nothing to publish, octree is empty", 100);

  ASSERT_EQ(entries.size(), 1U);
  EXPECT_EQ(entries[0].stamp_ns, 3);
  EXPECT_EQ(entries[0].repeat_count, 3U);

  entries = append_operator_log(entries, 40, 4, "planner", "planning failed", 100);
  ASSERT_EQ(entries.size(), 2U);
  EXPECT_EQ(entries[1].repeat_count, 1U);
}

TEST(PanelState, BuildsEmbeddedJointTestSampleInUrdfOrderAndRadians)
{
  const auto & specs = joint_test_specs();
  ASSERT_EQ(specs.size(), 4U);
  EXPECT_EQ(specs[0].name, "swing_joint");
  EXPECT_EQ(specs[0].lower_tick, -157);
  EXPECT_EQ(specs[0].upper_tick, 157);
  EXPECT_EQ(specs[1].name, "boom_joint");
  EXPECT_EQ(specs[2].name, "arm_joint");
  EXPECT_EQ(specs[3].name, "bucket_joint");

  const auto sample = make_joint_test_sample({10, -20, 30, -40});
  EXPECT_EQ(
    sample.names,
    (std::array<std::string, 4>{
      "swing_joint", "boom_joint", "arm_joint", "bucket_joint"}));
  EXPECT_EQ(sample.positions_rad, (std::array<double, 4>{0.1, -0.2, 0.3, -0.4}));
}

TEST(PanelState, AllowsEmbeddedJointTestsOnlyInExplicitIsolatedFixtureShadow)
{
  auto runtime = safe_runtime();
  EXPECT_TRUE(joint_test_publishing_allowed(runtime, true, 1));
  EXPECT_FALSE(joint_test_publishing_allowed(runtime, false, 1));
  EXPECT_FALSE(joint_test_publishing_allowed(runtime, true, 2));

  runtime.input_source = "live";
  EXPECT_FALSE(joint_test_publishing_allowed(runtime, true, 1));
  runtime = safe_runtime();
  runtime.fresh = false;
  EXPECT_FALSE(joint_test_publishing_allowed(runtime, true, 1));
  runtime = safe_runtime();
  runtime.motion_authorized = true;
  EXPECT_FALSE(joint_test_publishing_allowed(runtime, true, 1));
  runtime = safe_runtime();
  runtime.sender_constructed = true;
  EXPECT_FALSE(joint_test_publishing_allowed(runtime, true, 1));
  runtime = safe_runtime();
  runtime.action_datagrams = 1;
  EXPECT_FALSE(joint_test_publishing_allowed(runtime, true, 1));
}

}  // namespace
}  // namespace airy_mission_panel

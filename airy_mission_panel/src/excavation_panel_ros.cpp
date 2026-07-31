#include "airy_mission_panel/excavation_panel.hpp"

#include <array>
#include <cstdint>

#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSlider>

namespace airy_mission_panel
{

namespace
{
constexpr double kStatusMaxAgeS = 1.5;
constexpr std::size_t kMaxOperatorLogEntries = 100;

std::string target_topic(const std::string & phase)
{
  return "/mission/" + phase + "_target_snapshot";
}
}  // namespace

void ExcavationPanel::createRosInterfaces()
{
  plan_client_ = rclcpp_action::create_client<Plan>(node_, "/planning/plan");
  follow_client_ = rclcpp_action::create_client<Follow>(node_, "/excavator/follow");
  execute_dig_client_ =
    rclcpp_action::create_client<ExecuteDig>(node_, "/excavator/execute_dig");
  execute_dump_client_ =
    rclcpp_action::create_client<ExecuteDump>(node_, "/excavator/execute_dump");
  excavation_cycle_client_ =
    rclcpp_action::create_client<ExcavationCycle>(node_, "/mission/run_cycle");
  return_home_client_ =
    rclcpp_action::create_client<ReturnHome>(node_, "/excavator/return_home");

  auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  status_subscription_ = node_->create_subscription<
    airy_excavator_interfaces::msg::RuntimeStatus>(
    "/mission/runtime_status", latched_qos,
    [this, lifetime = callback_lifetime_](
      const airy_excavator_interfaces::msg::RuntimeStatus::SharedPtr message) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      runtime_.received = true;
      runtime_.input_source = message->input_source;
      runtime_.execution_mode = message->execution_mode;
      runtime_.control_stage = message->control_stage;
      runtime_.motion_backend = message->motion_backend;
      runtime_.motion_authorized = message->motion_authorized;
      runtime_.sender_constructed = message->sender_constructed;
      runtime_.quiescent = message->quiescent;
      runtime_.action_datagrams = message->action_datagrams;
      runtime_.active_behavior = message->active_behavior;
      runtime_.state_fresh = message->state_fresh;
      runtime_.control_enabled = message->control_enabled;
      runtime_.sensor_valid = message->sensor_valid;
      runtime_.stm32_alive = message->stm32_alive;
      runtime_.estop = message->estop;
      runtime_.fault_free = message->fault_free;
      runtime_.fixed_actions_validated = message->fixed_actions_validated;
      runtime_.follow_control_mode = message->follow_control_mode;
      runtime_.motion_gate_reason = message->motion_gate_reason;
      runtime_.last_rejection_reason = message->last_rejection_reason;
      runtime_.last_rejection_message = message->last_rejection_message;
      runtime_stamp_ = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
    });
  dig_subscription_ = node_->create_subscription<
    airy_excavator_interfaces::msg::TargetSnapshot>(
    target_topic("dig"), latched_qos,
    [this, lifetime = callback_lifetime_](
      const airy_excavator_interfaces::msg::TargetSnapshot::SharedPtr message) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      dig_target_ = message;
      dig_stamp_ = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
    });
  dump_subscription_ = node_->create_subscription<
    airy_excavator_interfaces::msg::TargetSnapshot>(
    target_topic("dump"), latched_qos,
    [this, lifetime = callback_lifetime_](
      const airy_excavator_interfaces::msg::TargetSnapshot::SharedPtr message) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      dump_target_ = message;
      dump_stamp_ = rclcpp::Time(message->header.stamp, RCL_ROS_TIME);
    });
  home_subscription_ = node_->create_subscription<
    airy_excavator_interfaces::msg::HomePoseCatalog>(
    "/mission/home_pose_catalog", latched_qos,
    [this, lifetime = callback_lifetime_](
      const airy_excavator_interfaces::msg::HomePoseCatalog::SharedPtr message) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      if (message->pose_ids.size() != message->pose_statuses.size()) {
        return;
      }
      home_pose_set_sha256_ = message->pose_set_sha256;
      home_pose_ids_ = message->pose_ids;
      home_pose_statuses_ = message->pose_statuses;
      ++home_catalog_revision_;
    });
  rosout_subscription_ = node_->create_subscription<rcl_interfaces::msg::Log>(
    "/rosout", rclcpp::RosoutQoS(),
    [this, lifetime = callback_lifetime_](const rcl_interfaces::msg::Log::SharedPtr message) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      const auto stamp_ns =
        static_cast<std::int64_t>(message->stamp.sec) * 1000000000LL +
        static_cast<std::int64_t>(message->stamp.nanosec);
      if (message->level < rcl_interfaces::msg::Log::WARN) {
        return;
      }
      std::scoped_lock lock(mutex_);
      operator_logs_ = append_operator_log(
        operator_logs_, message->level, stamp_ns, message->name, message->msg,
        kMaxOperatorLogEntries);
      ++operator_log_revision_;
    });
}

void ExcavationPanel::resetJointTests()
{
  const auto & specs = joint_test_specs();
  for (std::size_t index = 0; index < kJointTestCount; ++index) {
    const QSignalBlocker blocker(joint_test_sliders_[index]);
    joint_test_sliders_[index]->setValue(specs[index].default_tick);
    const auto radians = static_cast<double>(specs[index].default_tick) * 0.01;
    joint_test_value_labels_[index]->setText(
      QString("%1 rad / %2 deg").arg(radians, 0, 'f', 3).arg(
        radians * 57.2957795, 0, 'f', 1));
  }
  publishJointTestState(false);
}

void ExcavationPanel::publishJointTestState(bool require_continuous)
{
  if (!node_ || !joint_test_publisher_ ||
    (require_continuous && !joint_test_continuous_checkbox_->isChecked()))
  {
    return;
  }
  RuntimeSnapshot runtime;
  {
    std::scoped_lock lock(mutex_);
    runtime = runtime_;
    if (runtime.received && runtime_stamp_.nanoseconds() > 0) {
      const auto age = (node_->now() - runtime_stamp_).seconds();
      runtime.fresh = age >= 0.0 && age <= kStatusMaxAgeS;
    }
  }
  if (!joint_test_publishing_allowed(
      runtime, embedded_joint_tests_enabled_,
      node_->count_publishers(joint_test_publisher_->get_topic_name())))
  {
    return;
  }

  std::array<int, kJointTestCount> ticks;
  for (std::size_t index = 0; index < kJointTestCount; ++index) {
    ticks[index] = joint_test_sliders_[index]->value();
  }
  const auto sample = make_joint_test_sample(ticks);
  sensor_msgs::msg::JointState message;
  message.header.stamp = node_->now();
  message.name.assign(sample.names.begin(), sample.names.end());
  message.position.assign(sample.positions_rad.begin(), sample.positions_rad.end());
  joint_test_publisher_->publish(message);
  ++joint_test_publish_count_;
}

void ExcavationPanel::refreshJointTestControls(const RuntimeSnapshot & runtime)
{
  const auto publisher_count =
    node_ && joint_test_publisher_ ?
    node_->count_publishers(joint_test_publisher_->get_topic_name()) : 0;
  const bool allowed = joint_test_publishing_allowed(
    runtime, embedded_joint_tests_enabled_, publisher_count);
  for (auto * slider : joint_test_sliders_) {
    slider->setEnabled(allowed);
  }
  joint_test_continuous_checkbox_->setEnabled(allowed);
  joint_test_publish_button_->setEnabled(allowed);
  joint_test_reset_button_->setEnabled(allowed);

  if (!embedded_joint_tests_enabled_) {
    joint_test_status_label_->setText("DISABLED BY LAUNCH / safe for live inputs");
  } else if (publisher_count > 1) {
    joint_test_status_label_->setText("BLOCKED / multiple JointState publishers detected");
  } else if (!allowed) {
    joint_test_status_label_->setText("LOCKED / requires fresh FIXTURE + SHADOW + no-motion status");
  } else {
    joint_test_status_label_->setText(
      QString("READY / simulated %1 / published=%2")
      .arg(QString::fromStdString(joint_test_publisher_->get_topic_name()))
      .arg(joint_test_publish_count_));
  }
}

}  // namespace airy_mission_panel

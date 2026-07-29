#include "airy_mission_panel/excavation_panel.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

#include <QColor>
#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QPushButton>
#include <QSlider>
#include <QTabWidget>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace airy_mission_panel
{

namespace
{
constexpr double kStatusMaxAgeS = 1.5, kTargetMaxAgeS = 1.5;
QString severity_text(OperatorLogSeverity severity)
{
  switch (severity) {
    case OperatorLogSeverity::kWarning:
      return "WARN";
    case OperatorLogSeverity::kError:
      return "ERROR";
    case OperatorLogSeverity::kFatal:
      return "FATAL";
  }
  return "UNKNOWN";
}
QColor severity_color(OperatorLogSeverity severity)
{
  return severity == OperatorLogSeverity::kWarning ? QColor("#d99a2b") : QColor("#ef5350");
}
QString format_log_stamp(std::int64_t stamp_ns)
{
  std::ostringstream stream;
  stream << stamp_ns / 1000000000LL << '.';
  stream.width(3);
  stream.fill('0');
  stream << (stamp_ns % 1000000000LL) / 1000000LL;
  return QString::fromStdString(stream.str());
}
}  // namespace

ExcavationPanel::ExcavationPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * root_layout = new QVBoxLayout(this);
  safety_label_ = new QLabel("LOCKED / WAITING FOR SHADOW STATUS", this);
  safety_label_->setStyleSheet("font-weight: bold; color: #ff6666;");
  root_layout->addWidget(safety_label_);

  tabs_ = new QTabWidget(this);
  tabs_->setObjectName("mission_panel_tabs");
  root_layout->addWidget(tabs_);

  auto * actions_page = new QWidget(tabs_);
  auto * layout = new QVBoxLayout(actions_page);
  tabs_->addTab(actions_page, "Actions");

  auto * status_box = new QGroupBox("Runtime", this);
  auto * status_layout = new QFormLayout(status_box);
  runtime_label_ = new QLabel("-", status_box);
  operation_label_ = new QLabel("Idle", status_box);
  feedback_label_ = new QLabel("-", status_box);
  result_label_ = new QLabel("-", status_box);
  follow_status_label_ = new QLabel("FOLLOW LOCKED", status_box);
  follow_status_label_->setWordWrap(true);
  result_label_->setWordWrap(true);
  status_layout->addRow("Mode", runtime_label_);
  status_layout->addRow("Operation", operation_label_);
  status_layout->addRow("Feedback", feedback_label_);
  status_layout->addRow("Result", result_label_);
  status_layout->addRow("Follow", follow_status_label_);
  layout->addWidget(status_box);

  dig_button_ = new QPushButton("Plan + Follow DIG", this);
  dump_button_ = new QPushButton("Plan + Follow DUMP", this);
  dig_button_->setObjectName("plan_follow_dig");
  dump_button_->setObjectName("plan_follow_dump");
  layout->addWidget(dig_button_);
  layout->addWidget(dump_button_);

  home_pose_combo_ = new QComboBox(this);
  return_home_button_ = new QPushButton("Return Home", this);
  layout->addWidget(home_pose_combo_);
  layout->addWidget(return_home_button_);

  cancel_button_ = new QPushButton("Cancel Panel Operation (NOT E-STOP)", this);
  cancel_button_->setStyleSheet("color: #ffcc66;");
  layout->addWidget(cancel_button_);

  execute_dig_button_ = new QPushButton("ExecuteDig", this);
  execute_dump_button_ = new QPushButton("ExecuteDump", this);
  full_mission_button_ = new QPushButton("Full Mission", this);
  execute_dig_button_->setEnabled(false);
  execute_dump_button_->setEnabled(false);
  full_mission_button_->setEnabled(false);
  full_mission_button_->setToolTip(
    "Plan DIG -> Follow -> ExecuteDig -> re-plan DUMP -> Follow -> ExecuteDump");
  layout->addWidget(execute_dig_button_);
  layout->addWidget(execute_dump_button_);
  layout->addWidget(full_mission_button_);

  auto * logs_page = new QWidget(tabs_);
  auto * logs_page_layout = new QVBoxLayout(logs_page);
  log_box_ = new QGroupBox("Log History — Warnings / Errors (/rosout)", logs_page);
  auto * log_layout = new QVBoxLayout(log_box_);
  log_table_ = new QTableWidget(0, 5, log_box_);
  log_table_->setHorizontalHeaderLabels({"Time", "Level", "Count", "Module", "Message"});
  log_table_->horizontalHeader()->setStretchLastSection(true);
  log_table_->verticalHeader()->setVisible(false);
  log_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  log_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  log_table_->setColumnWidth(0, 90);
  log_table_->setColumnWidth(1, 55);
  log_table_->setColumnWidth(2, 50);
  log_table_->setColumnWidth(3, 140);
  log_table_->setMinimumHeight(80);
  log_table_->setToolTip(
    "Shows WARN, ERROR and FATAL messages from ROS /rosout. "
    "Process exits remain visible in the launch terminal and ~/.ros/log.");
  clear_log_button_ = new QPushButton("Clear", log_box_);
  log_layout->addWidget(log_table_);
  log_layout->addWidget(clear_log_button_);
  logs_page_layout->addWidget(log_box_);
  tabs_->addTab(logs_page, "Logs");

  auto * tests_page = new QWidget(tabs_);
  auto * tests_layout = new QVBoxLayout(tests_page);
  auto * tests_warning = new QLabel(
    "TEST ONLY — fixture sliders are no-motion; Live Hold-to-Jog sends bounded real commands.",
    tests_page);
  tests_warning->setWordWrap(true);
  tests_warning->setStyleSheet("font-weight: bold; color: #ff9966;");
  tests_layout->addWidget(tests_warning);
  joint_test_status_label_ = new QLabel("LOCKED / waiting for initialization", tests_page);
  tests_layout->addWidget(joint_test_status_label_);
  joint_test_continuous_checkbox_ = new QCheckBox("Publish continuously at 10 Hz", tests_page);
  joint_test_continuous_checkbox_->setChecked(true);
  tests_layout->addWidget(joint_test_continuous_checkbox_);
  auto * joint_grid = new QGridLayout();
  const auto & specs = joint_test_specs();
  for (std::size_t index = 0; index < kJointTestCount; ++index) {
    const auto & spec = specs[index];
    auto * slider = new QSlider(Qt::Horizontal, tests_page);
    slider->setObjectName(QString("joint_test_slider_%1").arg(QString::fromStdString(spec.name)));
    slider->setRange(spec.lower_tick, spec.upper_tick);
    slider->setValue(spec.default_tick);
    auto * value_label = new QLabel("0.000 rad / 0.0 deg", tests_page);
    connect(slider, &QSlider::valueChanged, this, [this, value_label](int tick) {
      const auto radians = static_cast<double>(tick) * 0.01;
      value_label->setText(
        QString("%1 rad / %2 deg").arg(radians, 0, 'f', 3).arg(radians * 57.2957795, 0, 'f', 1));
      publishJointTestState(false);
    });
    joint_test_sliders_[index] = slider;
    joint_test_value_labels_[index] = value_label;
    joint_grid->addWidget(new QLabel(QString::fromStdString(spec.name), tests_page), index, 0);
    joint_grid->addWidget(slider, index, 1);
    joint_grid->addWidget(value_label, index, 2);
  }
  tests_layout->addLayout(joint_grid);
  joint_test_publish_button_ = new QPushButton("Publish Once", tests_page);
  joint_test_publish_button_->setObjectName("joint_test_publish");
  connect(
    joint_test_publish_button_, &QPushButton::clicked,
    this, [this]() {publishJointTestState(false);});
  joint_test_reset_button_ = new QPushButton("Reset to Zero", tests_page);
  joint_test_reset_button_->setObjectName("joint_test_reset");
  connect(
    joint_test_reset_button_, &QPushButton::clicked,
    this, &ExcavationPanel::resetJointTests);
  tests_layout->addWidget(joint_test_publish_button_);
  tests_layout->addWidget(joint_test_reset_button_);

  auto * manual_jog_box = new QGroupBox("Live Hold-to-Jog — low speed", tests_page);
  auto * manual_jog_layout = new QGridLayout(manual_jog_box);
  manual_jog_status_label_ = new QLabel(
    "LOCKED / requires live control and fresh machine state", manual_jog_box);
  manual_jog_status_label_->setWordWrap(true);
  manual_jog_layout->addWidget(manual_jog_status_label_, 0, 0, 1, 3);
  const std::array<std::string, 3> jog_actuators{{"boom", "stick", "bucket"}};
  for (std::size_t index = 0; index < jog_actuators.size(); ++index) {
    const auto & actuator = jog_actuators[index];
    auto * negative = new QPushButton("Action −", manual_jog_box);
    auto * positive = new QPushButton("Action +", manual_jog_box);
    negative->setObjectName(QString("manual_jog_%1_negative").arg(
        QString::fromStdString(actuator)));
    positive->setObjectName(QString("manual_jog_%1_positive").arg(
        QString::fromStdString(actuator)));
    negative->setToolTip("Hold to send a negative action; release stops");
    positive->setToolTip("Hold to send a positive action; release stops");
    manual_jog_buttons_[index * 2] = negative;
    manual_jog_buttons_[index * 2 + 1] = positive;
    manual_jog_layout->addWidget(
      new QLabel(QString::fromStdString(actuator), manual_jog_box), index + 1, 0);
    manual_jog_layout->addWidget(negative, index + 1, 1);
    manual_jog_layout->addWidget(positive, index + 1, 2);
    connect(negative, &QPushButton::pressed, this, [this, actuator, negative]() {
      startManualJog(actuator, -1, negative);
    });
    connect(positive, &QPushButton::pressed, this, [this, actuator, positive]() {
      startManualJog(actuator, 1, positive);
    });
    connect(negative, &QPushButton::released, this, &ExcavationPanel::stopManualJog);
    connect(positive, &QPushButton::released, this, &ExcavationPanel::stopManualJog);
  }
  tests_layout->addWidget(manual_jog_box);
  tests_layout->addStretch();
  tabs_->addTab(tests_page, "Tests");
  layout->addStretch();

  connect(dig_button_, &QPushButton::clicked, this, &ExcavationPanel::startDig);
  connect(dump_button_, &QPushButton::clicked, this, &ExcavationPanel::startDump);
  connect(
    return_home_button_, &QPushButton::clicked,
    this, &ExcavationPanel::startReturnHome);
  connect(execute_dig_button_, &QPushButton::clicked, this, &ExcavationPanel::startExecuteDig);
  connect(execute_dump_button_, &QPushButton::clicked, this, &ExcavationPanel::startExecuteDump);
  connect(full_mission_button_, &QPushButton::clicked, this, &ExcavationPanel::startFullMission);
  connect(
    cancel_button_, &QPushButton::clicked,
    this, &ExcavationPanel::cancelOwnedOperation);
  connect(clear_log_button_, &QPushButton::clicked, this, &ExcavationPanel::clearLogs);

  refresh_timer_ = new QTimer(this);
  connect(refresh_timer_, &QTimer::timeout, this, &ExcavationPanel::refreshView);
  refresh_timer_->start(100);
  jog_heartbeat_timer_ = new QTimer(this);
  jog_heartbeat_timer_->setInterval(50);
  connect(
    jog_heartbeat_timer_, &QTimer::timeout,
    this, &ExcavationPanel::publishJogHeartbeat);
  operator_heartbeat_timer_ = new QTimer(this);
  operator_heartbeat_timer_->setInterval(50);
  connect(
    operator_heartbeat_timer_, &QTimer::timeout,
    this, &ExcavationPanel::publishOperatorHeartbeat);
  refreshView();
}

ExcavationPanel::~ExcavationPanel()
{
  refresh_timer_->stop();
  jog_heartbeat_timer_->stop();
  operator_heartbeat_timer_->stop();
  {
    std::unique_lock lock(callback_lifetime_->mutex);
    callback_lifetime_->alive = false;
  }
  status_subscription_.reset();
  dig_subscription_.reset();
  dump_subscription_.reset();
  home_subscription_.reset();
  rosout_subscription_.reset();
  joint_test_publisher_.reset();
  plan_client_.reset();
  follow_client_.reset();
  execute_dig_client_.reset();
  execute_dump_client_.reset();
  excavation_cycle_client_.reset();
  return_home_client_.reset();
  hold_to_jog_client_.reset();
  jog_heartbeat_publisher_.reset();
  operator_heartbeat_publisher_.reset();
}

void ExcavationPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    std::scoped_lock lock(mutex_);
    failOperationLocked("RViz ROS node is unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  if (!node_->has_parameter("enable_embedded_joint_tests")) {
    node_->declare_parameter<bool>("enable_embedded_joint_tests", false);
  }
  embedded_joint_tests_enabled_ =
    node_->get_parameter("enable_embedded_joint_tests").as_bool();
  if (embedded_joint_tests_enabled_) {
    joint_test_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  }
  createRosInterfaces();
}

void ExcavationPanel::startDig()
{
  startClickedPlanFollow("dig");
}

void ExcavationPanel::startDump()
{
  startClickedPlanFollow("dump");
}

void ExcavationPanel::startClickedPlanFollow(const std::string & phase)
{
  bool supervised_canary = false;
  {
    std::scoped_lock lock(mutex_);
    if (owned_operation_ != OwnedOperation::kIdle) {
      return;
    }
    supervised_canary = runtime_.follow_control_mode == "supervised_canary";
    if (supervised_canary) {
      follow_heartbeat_active_ = true;
      follow_session_id_.clear();
    }
  }
  if (supervised_canary) {
    operator_heartbeat_timer_->start();
  }
  startPlanFollow(phase);
  bool heartbeat_still_active = false;
  {
    std::scoped_lock lock(mutex_);
    if (owned_operation_ != OwnedOperation::kPlanFollow) {
      follow_heartbeat_active_ = false;
    }
    heartbeat_still_active = follow_heartbeat_active_;
  }
  if (!heartbeat_still_active) {
    operator_heartbeat_timer_->stop();
  }
}

void ExcavationPanel::publishOperatorHeartbeat()
{
  std::string session_id;
  {
    std::scoped_lock lock(mutex_);
    if (!follow_heartbeat_active_) {
      operator_heartbeat_timer_->stop();
      return;
    }
    session_id = follow_session_id_;
  }
  if (session_id.empty()) {return;}
  if (!node_ || !operator_heartbeat_publisher_) {
    cancelOwnedOperation();
    return;
  }
  airy_excavator_interfaces::msg::OperatorHeartbeat heartbeat;
  heartbeat.header.stamp = node_->now();
  heartbeat.behavior = "Follow";
  heartbeat.session_id = session_id;
  operator_heartbeat_publisher_->publish(heartbeat);
}

void ExcavationPanel::startManualJog(
  const std::string & actuator, int direction, QPushButton * button)
{
  HoldToJog::Goal goal;
  {
    std::scoped_lock lock(mutex_);
    if (
      !node_ || !button || !panelViewLocked(node_->now()).manual_jog_enabled ||
      !hold_to_jog_client_ || !hold_to_jog_client_->action_server_is_ready())
    {
      return;
    }
    jog_session_id_ = "rviz_jog_" + std::to_string(node_->now().nanoseconds());
    goal.session_id = jog_session_id_;
    goal.actuator = actuator;
    goal.direction = static_cast<std::int8_t>(direction);
    owned_operation_ = OwnedOperation::kManualJog;
    active_manual_jog_button_ = button;
    jog_heartbeat_active_ = true;
    cancel_requested_ = false;
    operation_text_ = "Holding manual jog: " + actuator +
      (direction > 0 ? " action +" : " action -");
    feedback_text_ = "Waiting for HoldToJog goal response";
    result_text_ = "Release the button to stop";
  }
  publishJogHeartbeat();
  jog_heartbeat_timer_->start();

  rclcpp_action::Client<HoldToJog>::SendGoalOptions options;
  options.goal_response_callback =
    [this, lifetime = callback_lifetime_](HoldToJogGoalHandle::SharedPtr handle) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      bool cancel = false;
      {
        std::scoped_lock lock(mutex_);
        if (!handle) {
          jog_heartbeat_active_ = false;
          failOperationLocked("HoldToJog goal rejected");
          return;
        }
        hold_to_jog_goal_handle_ = handle;
        feedback_text_ = "HoldToJog accepted; release to stop";
        cancel = cancel_requested_;
      }
      if (cancel) {hold_to_jog_client_->async_cancel_goal(handle);}
    };
  options.feedback_callback =
    [this, lifetime = callback_lifetime_](
    HoldToJogGoalHandle::SharedPtr,
    const std::shared_ptr<const HoldToJog::Feedback> feedback) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      std::scoped_lock lock(mutex_);
      feedback_text_ = feedback->actuator + " pos=" +
        std::to_string(feedback->position_m).substr(0, 7) + " m / cmd=" +
        std::to_string(feedback->commanded_velocity).substr(0, 7) + " / datagrams=" +
        std::to_string(feedback->action_datagrams);
    };
  options.result_callback =
    [this, lifetime = callback_lifetime_](
    const HoldToJogGoalHandle::WrappedResult & wrapped) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      std::scoped_lock lock(mutex_);
      jog_heartbeat_active_ = false;
      hold_to_jog_goal_handle_.reset();
      if (
        wrapped.result && wrapped.result->quiescence_confirmed &&
        ((wrapped.code == rclcpp_action::ResultCode::CANCELED &&
        wrapped.result->outcome == HoldToJog::Result::OUTCOME_CANCELLED &&
        wrapped.result->reason_code == "CANCELLED") ||
        (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
        wrapped.result->outcome == HoldToJog::Result::OUTCOME_SUCCEEDED &&
        wrapped.result->reason_code == "MAX_HOLD_REACHED")))
      {
        finishOperationLocked(
          "HoldToJog stopped safely / " + wrapped.result->reason_code +
          " / before=" + std::to_string(wrapped.result->initial_position_m).substr(0, 8) +
          " m / after=" + std::to_string(wrapped.result->final_position_m).substr(0, 8) +
          " m / delta=" + std::to_string(wrapped.result->position_delta_m).substr(0, 9) +
          " m / datagrams=" + std::to_string(wrapped.result->action_datagrams));
      } else {
        failOperationLocked(
          wrapped.result ? "HoldToJog stopped: " + wrapped.result->reason_code :
          "HoldToJog failed without Result");
      }
    };
  hold_to_jog_client_->async_send_goal(goal, options);
}

void ExcavationPanel::stopManualJog()
{
  HoldToJogGoalHandle::SharedPtr handle;
  {
    std::scoped_lock lock(mutex_);
    if (owned_operation_ != OwnedOperation::kManualJog) {return;}
    jog_heartbeat_active_ = false;
    cancel_requested_ = true;
    feedback_text_ = "Released; requesting terminal zero";
    handle = hold_to_jog_goal_handle_;
  }
  jog_heartbeat_timer_->stop();
  if (handle) {hold_to_jog_client_->async_cancel_goal(handle);}
}

void ExcavationPanel::publishJogHeartbeat()
{
  std::string session_id;
  QPushButton * active_button = nullptr;
  {
    std::scoped_lock lock(mutex_);
    if (!jog_heartbeat_active_) {
      jog_heartbeat_timer_->stop();
      return;
    }
    session_id = jog_session_id_;
    active_button = active_manual_jog_button_;
  }
  if (!active_button || !active_button->isDown() || !window()->isActiveWindow()) {
    stopManualJog();
    return;
  }
  if (!node_ || !jog_heartbeat_publisher_) {stopManualJog(); return;}
  airy_excavator_interfaces::msg::JogHeartbeat heartbeat;
  heartbeat.header.stamp = node_->now();
  heartbeat.session_id = session_id;
  jog_heartbeat_publisher_->publish(heartbeat);
}

void ExcavationPanel::startExecuteDig()
{
  sendExecute("dig");
}

void ExcavationPanel::startExecuteDump()
{
  sendExecute("dump");
}

void ExcavationPanel::startFullMission()
{
  ExcavationCycle::Goal goal;
  {
    std::scoped_lock lock(mutex_);
    if (
      !node_ || !dig_target_ || !dump_target_ ||
      !panelViewLocked(node_->now()).full_mission_enabled)
    {
      return;
    }
    goal.dig_target = *dig_target_;
    goal.dump_target = *dump_target_;
    owned_operation_ = OwnedOperation::kFullMission;
    active_phase_ = "mission";
    cancel_requested_ = false;
    operation_text_ = "Running Full Mission";
    feedback_text_ = "Waiting for Mission goal response";
    result_text_ = "-";
  }
  rclcpp_action::Client<ExcavationCycle>::SendGoalOptions options;
  options.goal_response_callback =
    [this, lifetime = callback_lifetime_](ExcavationCycleGoalHandle::SharedPtr handle) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      bool cancel = false;
      {
        std::scoped_lock lock(mutex_);
        if (!handle) {failOperationLocked("Full Mission goal rejected"); return;}
        excavation_cycle_goal_handle_ = handle;
        feedback_text_ = "Full Mission goal accepted";
        cancel = cancel_requested_;
      }
      if (cancel) {excavation_cycle_client_->async_cancel_goal(handle);}
    };
  options.feedback_callback =
    [this, lifetime = callback_lifetime_](
    ExcavationCycleGoalHandle::SharedPtr,
    const std::shared_ptr<const ExcavationCycle::Feedback> feedback) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      std::scoped_lock lock(mutex_);
      feedback_text_ = feedback->stage + " / " + feedback->message;
    };
  options.result_callback =
    [this, lifetime = callback_lifetime_](
    const ExcavationCycleGoalHandle::WrappedResult & wrapped) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      std::scoped_lock lock(mutex_);
      excavation_cycle_goal_handle_.reset();
      if (
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
        wrapped.result->outcome == ExcavationCycle::Result::OUTCOME_SUCCEEDED &&
        wrapped.result->reason_code == "SUCCEEDED" && wrapped.result->quiescence_confirmed)
      {
        finishOperationLocked(
          "Full Mission SUCCEEDED / datagrams=" +
          std::to_string(wrapped.result->action_datagrams));
      } else if (
        cancel_requested_ && wrapped.code == rclcpp_action::ResultCode::CANCELED &&
        wrapped.result && wrapped.result->outcome == ExcavationCycle::Result::OUTCOME_CANCELLED &&
        wrapped.result->quiescence_confirmed)
      {
        finishOperationLocked("Full Mission CANCELLED");
      } else {
        failOperationLocked(
          wrapped.result ? "Full Mission failed: " + wrapped.result->message :
          "Full Mission failed without Result");
      }
    };
  excavation_cycle_client_->async_send_goal(goal, options);
}

void ExcavationPanel::startPlanFollow(const std::string & phase)
{
  airy_excavator_interfaces::msg::TargetSnapshot target;
  std::string planning_scope;
  {
    std::scoped_lock lock(mutex_);
    if (!node_) {
      return;
    }
    const auto view = panelViewLocked(node_->now());
    const bool enabled =
      phase == "dig" ? view.plan_follow_dig_enabled : view.plan_follow_dump_enabled;
    const auto source = phase == "dig" ? dig_target_ : dump_target_;
    if (!enabled || !source) {
      return;
    }
    target = *source;
    owned_operation_ = OwnedOperation::kPlanFollow;
    active_phase_ = phase;
    cancel_requested_ = false;
    operation_text_ = "Planning " + phase;
    feedback_text_ = "Waiting for Plan goal response";
    result_text_ = "-";
    planning_scope = runtime_.execution_mode == "control" ?
      "execution_strict" : "preview_global";
  }

  Plan::Goal goal;
  goal.target = target;
  goal.planning_scope = planning_scope;
  rclcpp_action::Client<Plan>::SendGoalOptions options;
  options.goal_response_callback =
    [this, lifetime = callback_lifetime_](PlanGoalHandle::SharedPtr handle) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      bool cancel = false;
      {
        std::scoped_lock lock(mutex_);
        if (!handle) {
          failOperationLocked("Plan goal rejected");
          return;
        }
        plan_goal_handle_ = handle;
        feedback_text_ = "Plan goal accepted";
        cancel = cancel_requested_;
      }
      if (cancel) {
        plan_client_->async_cancel_goal(
          handle,
          [this, lifetime = callback_lifetime_](const auto & response) {
            std::shared_lock lifetime_lock(lifetime->mutex);
            if (lifetime->alive) {
              observeCancelResponse(response);
            }
          });
      }
    };
  options.feedback_callback =
    [this, lifetime = callback_lifetime_](
    PlanGoalHandle::SharedPtr, const std::shared_ptr<const Plan::Feedback> feedback) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      feedback_text_ = "Plan: " + feedback->stage;
    };
  options.result_callback =
    [this, target, planning_scope, lifetime = callback_lifetime_](
    const PlanGoalHandle::WrappedResult & wrapped) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      airy_excavator_interfaces::msg::TrajectorySnapshot trajectory;
      {
        std::scoped_lock lock(mutex_);
        plan_goal_handle_.reset();
        if (cancel_requested_) {
          if (
            wrapped.code == rclcpp_action::ResultCode::CANCELED && wrapped.result &&
            wrapped.result->outcome == Plan::Result::OUTCOME_CANCELLED &&
            wrapped.result->reason_code == "CANCELLED" &&
            wrapped.result->action_datagrams == 0)
          {
            finishOperationLocked("Plan CANCELLED / Follow not started");
          } else if (
            wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
            wrapped.result->outcome == Plan::Result::OUTCOME_SUCCEEDED &&
            wrapped.result->reason_code == "SUCCEEDED" &&
            wrapped.result->action_datagrams == 0)
          {
            finishOperationLocked("Plan completed / Follow suppressed by Cancel");
          } else {
            failOperationLocked("Cancel requested; Plan terminal state was not safe");
          }
          return;
        }
        if (
          wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
          wrapped.result->outcome != Plan::Result::OUTCOME_SUCCEEDED ||
          wrapped.result->reason_code != "SUCCEEDED" ||
          wrapped.result->action_datagrams != 0)
        {
          failOperationLocked(
            wrapped.result ? "Plan failed: " + wrapped.result->reason_code :
            "Plan failed without Result");
          return;
        }
        trajectory = wrapped.result->trajectory;
        const auto valid_until = rclcpp::Time(trajectory.valid_until, RCL_ROS_TIME);
        double endpoint_error_m = std::numeric_limits<double>::infinity();
        if (!trajectory.waypoints.empty()) {
          const auto & endpoint = trajectory.waypoints.back();
          endpoint_error_m = std::hypot(
            endpoint.x - target.position.x,
            endpoint.y - target.position.y,
            endpoint.z - target.position.z);
        }
        if (
          trajectory.header.frame_id != "machine_root_ros" ||
          trajectory.mission_id != target.mission_id ||
          trajectory.mission_sha256 != target.mission_sha256 ||
          trajectory.mission_phase != target.mission_phase ||
          trajectory.planning_scope != planning_scope ||
          trajectory.execution_eligible != (planning_scope == "execution_strict") ||
          trajectory.input_source != runtime_.input_source ||
          endpoint_error_m > target.radius_m ||
          valid_until <= node_->now())
        {
          failOperationLocked("Plan Result provenance/scope mismatch");
          return;
        }
        operation_text_ = "Starting Follow " + active_phase_;
      }
      sendFollow(trajectory);
    };
  plan_client_->async_send_goal(goal, options);
}

void ExcavationPanel::sendFollow(
  const airy_excavator_interfaces::msg::TrajectorySnapshot & trajectory)
{
  if (!follow_client_ || !follow_client_->action_server_is_ready()) {
    std::scoped_lock lock(mutex_);
    failOperationLocked("Follow Action Server unavailable");
    return;
  }
  Follow::Goal goal;
  goal.trajectory = trajectory;
  {
    std::scoped_lock lock(mutex_);
    if (runtime_.follow_control_mode == "supervised_canary") {
      if (!follow_heartbeat_active_) {
        failOperationLocked("Follow supervision was released before execution");
        return;
      }
      follow_session_id_ = trajectory.trajectory_id;
    }
  }
  publishOperatorHeartbeat();
  rclcpp_action::Client<Follow>::SendGoalOptions options;
  options.goal_response_callback =
    [this, lifetime = callback_lifetime_](FollowGoalHandle::SharedPtr handle) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      bool cancel = false;
      {
        std::scoped_lock lock(mutex_);
        if (!handle) {
          failOperationLocked("Follow goal rejected");
          return;
        }
        follow_goal_handle_ = handle;
        operation_text_ = "Following " + active_phase_;
        cancel = cancel_requested_;
      }
      if (cancel) {
        follow_client_->async_cancel_goal(
          handle,
          [this, lifetime = callback_lifetime_](const auto & response) {
            std::shared_lock lifetime_lock(lifetime->mutex);
            if (lifetime->alive) {
              observeCancelResponse(response);
            }
          });
      }
    };
  options.feedback_callback =
    [this, lifetime = callback_lifetime_](
    FollowGoalHandle::SharedPtr, const std::shared_ptr<const Follow::Feedback> feedback) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      feedback_text_ =
        "Waypoint " + std::to_string(feedback->current_waypoint_index + 1) + "/" +
        std::to_string(feedback->waypoint_count) + "  distance=" +
        std::to_string(feedback->distance_m).substr(0, 5) + " m";
    };
  options.result_callback =
    [this, lifetime = callback_lifetime_](const FollowGoalHandle::WrappedResult & wrapped) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      follow_goal_handle_.reset();
      if (
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
        wrapped.result->outcome == Follow::Result::OUTCOME_SUCCEEDED &&
        wrapped.result->reason_code == "SUCCEEDED" &&
        wrapped.result->quiescence_confirmed)
      {
        finishOperationLocked(
          "Follow SUCCEEDED / action_datagrams=" +
          std::to_string(wrapped.result->action_datagrams));
      } else if (
        cancel_requested_ && wrapped.code == rclcpp_action::ResultCode::CANCELED &&
        wrapped.result && wrapped.result->outcome == Follow::Result::OUTCOME_CANCELLED &&
        wrapped.result->reason_code == "CANCELLED" &&
        wrapped.result->quiescence_confirmed)
      {
        finishOperationLocked(
          "Follow CANCELLED / action_datagrams=" +
          std::to_string(wrapped.result->action_datagrams));
      } else if (
        wrapped.result && wrapped.result->quiescence_confirmed &&
        (wrapped.result->reason_code == "CANARY_TIME_LIMIT_REACHED" ||
        wrapped.result->reason_code == "SUPERVISION_HEARTBEAT_TIMEOUT"))
      {
        finishOperationLocked(
          "Follow CANARY STOPPED / " + wrapped.result->reason_code +
          " / action_datagrams=" + std::to_string(wrapped.result->action_datagrams));
      } else {
        failOperationLocked(
          wrapped.result ? "Follow failed: " + wrapped.result->reason_code :
          "Follow failed without Result");
      }
    };
  follow_client_->async_send_goal(goal, options);
}

void ExcavationPanel::sendExecute(const std::string & phase)
{
  airy_excavator_interfaces::msg::TargetSnapshot target;
  {
    std::scoped_lock lock(mutex_);
    if (!node_) {
      return;
    }
    const auto view = panelViewLocked(node_->now());
    const bool enabled = phase == "dig" ? view.execute_dig_enabled : view.execute_dump_enabled;
    const auto source = phase == "dig" ? dig_target_ : dump_target_;
    if (!enabled || !source) {
      return;
    }
    target = *source;
    owned_operation_ =
      phase == "dig" ? OwnedOperation::kExecuteDig : OwnedOperation::kExecuteDump;
    active_phase_ = phase;
    cancel_requested_ = false;
    operation_text_ = phase == "dig" ? "Executing DIG" : "Executing DUMP";
    feedback_text_ = "Waiting for fixed-action goal response";
    result_text_ = "-";
  }

  if (phase == "dig") {
    ExecuteDig::Goal goal;
    goal.target = target;
    rclcpp_action::Client<ExecuteDig>::SendGoalOptions options;
    options.goal_response_callback =
      [this, lifetime = callback_lifetime_](ExecuteDigGoalHandle::SharedPtr handle) {
        std::shared_lock lifetime_lock(lifetime->mutex);
        if (!lifetime->alive) {return;}
        bool cancel = false;
        {
          std::scoped_lock lock(mutex_);
          if (!handle) {failOperationLocked("ExecuteDig goal rejected"); return;}
          execute_dig_goal_handle_ = handle;
          feedback_text_ = "ExecuteDig goal accepted";
          cancel = cancel_requested_;
        }
        if (cancel) {
          execute_dig_client_->async_cancel_goal(handle);
        }
      };
    options.feedback_callback =
      [this, lifetime = callback_lifetime_](
      ExecuteDigGoalHandle::SharedPtr,
      const std::shared_ptr<const ExecuteDig::Feedback> feedback) {
        std::shared_lock lifetime_lock(lifetime->mutex);
        if (!lifetime->alive) {return;}
        std::scoped_lock lock(mutex_);
        feedback_text_ = feedback->step_label + " / " + feedback->phase +
          " / error=" + std::to_string(feedback->max_error).substr(0, 5);
      };
    options.result_callback =
      [this, lifetime = callback_lifetime_](const ExecuteDigGoalHandle::WrappedResult & wrapped) {
        std::shared_lock lifetime_lock(lifetime->mutex);
        if (!lifetime->alive) {return;}
        std::scoped_lock lock(mutex_);
        execute_dig_goal_handle_.reset();
        if (
          wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
          wrapped.result->outcome == ExecuteDig::Result::OUTCOME_SUCCEEDED &&
          wrapped.result->reason_code == "SEQUENCE_COMPLETED" &&
          wrapped.result->quiescence_confirmed)
        {
          finishOperationLocked(
            "ExecuteDig SEQUENCE_COMPLETED / datagrams=" +
            std::to_string(wrapped.result->action_datagrams));
        } else if (
          cancel_requested_ && wrapped.code == rclcpp_action::ResultCode::CANCELED &&
          wrapped.result && wrapped.result->outcome == ExecuteDig::Result::OUTCOME_CANCELLED &&
          wrapped.result->quiescence_confirmed)
        {
          finishOperationLocked("ExecuteDig CANCELLED");
        } else {
          failOperationLocked(
            wrapped.result ?
            "ExecuteDig failed: " + wrapped.result->reason_code + " / " +
            wrapped.result->message :
            "ExecuteDig failed without Result");
        }
      };
    execute_dig_client_->async_send_goal(goal, options);
    return;
  }

  ExecuteDump::Goal goal;
  goal.target = target;
  rclcpp_action::Client<ExecuteDump>::SendGoalOptions options;
  options.goal_response_callback =
    [this, lifetime = callback_lifetime_](ExecuteDumpGoalHandle::SharedPtr handle) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      bool cancel = false;
      {
        std::scoped_lock lock(mutex_);
        if (!handle) {failOperationLocked("ExecuteDump goal rejected"); return;}
        execute_dump_goal_handle_ = handle;
        feedback_text_ = "ExecuteDump goal accepted";
        cancel = cancel_requested_;
      }
      if (cancel) {
        execute_dump_client_->async_cancel_goal(handle);
      }
    };
  options.feedback_callback =
    [this, lifetime = callback_lifetime_](
    ExecuteDumpGoalHandle::SharedPtr,
    const std::shared_ptr<const ExecuteDump::Feedback> feedback) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      std::scoped_lock lock(mutex_);
      feedback_text_ = feedback->step_label + " / " + feedback->phase +
        " / error=" + std::to_string(feedback->max_error).substr(0, 5);
    };
  options.result_callback =
    [this, lifetime = callback_lifetime_](const ExecuteDumpGoalHandle::WrappedResult & wrapped) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {return;}
      std::scoped_lock lock(mutex_);
      execute_dump_goal_handle_.reset();
      if (
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
        wrapped.result->outcome == ExecuteDump::Result::OUTCOME_SUCCEEDED &&
        wrapped.result->reason_code == "SEQUENCE_COMPLETED" &&
        wrapped.result->quiescence_confirmed)
      {
        finishOperationLocked(
          "ExecuteDump SEQUENCE_COMPLETED / datagrams=" +
          std::to_string(wrapped.result->action_datagrams));
      } else if (
        cancel_requested_ && wrapped.code == rclcpp_action::ResultCode::CANCELED &&
        wrapped.result && wrapped.result->outcome == ExecuteDump::Result::OUTCOME_CANCELLED &&
        wrapped.result->quiescence_confirmed)
      {
        finishOperationLocked("ExecuteDump CANCELLED");
    } else {
      failOperationLocked(
        wrapped.result ?
        "ExecuteDump failed: " + wrapped.result->reason_code + " / " +
        wrapped.result->message :
        "ExecuteDump failed without Result");
    }
    };
  execute_dump_client_->async_send_goal(goal, options);
}

void ExcavationPanel::startReturnHome()
{
  ReturnHome::Goal goal;
  {
    std::scoped_lock lock(mutex_);
    const auto index = home_pose_combo_->currentIndex();
    if (
      !node_ || !panelViewLocked(node_->now()).return_home_enabled || index < 0 ||
      static_cast<std::size_t>(index) >= home_pose_ids_.size() ||
      home_pose_set_sha256_.empty())
    {
      return;
    }
    goal.home_pose_id = home_pose_ids_[index];
    goal.pose_set_sha256 = home_pose_set_sha256_;
    owned_operation_ = OwnedOperation::kReturnHome;
    cancel_requested_ = false;
    operation_text_ = "Returning Home: " + goal.home_pose_id;
    feedback_text_ = "Waiting for ReturnHome goal response";
    result_text_ = "-";
  }
  rclcpp_action::Client<ReturnHome>::SendGoalOptions options;
  options.goal_response_callback =
    [this, lifetime = callback_lifetime_](ReturnHomeGoalHandle::SharedPtr handle) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      bool cancel = false;
      {
        std::scoped_lock lock(mutex_);
        if (!handle) {
          failOperationLocked("ReturnHome goal rejected");
          return;
        }
        return_home_goal_handle_ = handle;
        feedback_text_ = "ReturnHome goal accepted";
        cancel = cancel_requested_;
      }
      if (cancel) {
        return_home_client_->async_cancel_goal(
          handle,
          [this, lifetime = callback_lifetime_](const auto & response) {
            std::shared_lock lifetime_lock(lifetime->mutex);
            if (lifetime->alive) {
              observeCancelResponse(response);
            }
          });
      }
    };
  options.feedback_callback =
    [this, lifetime = callback_lifetime_](
    ReturnHomeGoalHandle::SharedPtr,
    const std::shared_ptr<const ReturnHome::Feedback> feedback) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      feedback_text_ =
        "Home error=" + std::to_string(feedback->max_error_rad).substr(0, 5) + " rad";
    };
  options.result_callback =
    [this, lifetime = callback_lifetime_](
    const ReturnHomeGoalHandle::WrappedResult & wrapped) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      std::scoped_lock lock(mutex_);
      return_home_goal_handle_.reset();
      if (
        wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
        wrapped.result->outcome == ReturnHome::Result::OUTCOME_SUCCEEDED &&
        wrapped.result->reason_code == "SUCCEEDED" &&
        wrapped.result->quiescence_confirmed &&
        wrapped.result->action_datagrams == 0)
      {
        finishOperationLocked("ReturnHome SUCCEEDED / action_datagrams=0");
      } else if (
        cancel_requested_ && wrapped.code == rclcpp_action::ResultCode::CANCELED &&
        wrapped.result && wrapped.result->outcome == ReturnHome::Result::OUTCOME_CANCELLED &&
        wrapped.result->reason_code == "CANCELLED" &&
        wrapped.result->quiescence_confirmed && wrapped.result->action_datagrams == 0)
      {
        finishOperationLocked("ReturnHome CANCELLED / action_datagrams=0");
      } else {
        failOperationLocked(
          wrapped.result ? "ReturnHome failed: " + wrapped.result->reason_code :
          "ReturnHome failed without Result");
      }
    };
  return_home_client_->async_send_goal(goal, options);
}

void ExcavationPanel::cancelOwnedOperation()
{
  FollowGoalHandle::SharedPtr follow;
  ExecuteDigGoalHandle::SharedPtr execute_dig;
  ExecuteDumpGoalHandle::SharedPtr execute_dump;
  ExcavationCycleGoalHandle::SharedPtr excavation_cycle;
  PlanGoalHandle::SharedPtr plan;
  ReturnHomeGoalHandle::SharedPtr return_home;
  HoldToJogGoalHandle::SharedPtr hold_to_jog;
  {
    std::scoped_lock lock(mutex_);
    if (owned_operation_ == OwnedOperation::kIdle) {
      return;
    }
    cancel_requested_ = true;
    operation_text_ = "Cancelling Panel-owned operation";
    follow = follow_goal_handle_;
    execute_dig = execute_dig_goal_handle_;
    execute_dump = execute_dump_goal_handle_;
    excavation_cycle = excavation_cycle_goal_handle_;
    plan = plan_goal_handle_;
    return_home = return_home_goal_handle_;
    hold_to_jog = hold_to_jog_goal_handle_;
    if (owned_operation_ == OwnedOperation::kManualJog) {
      jog_heartbeat_active_ = false;
    }
    if (owned_operation_ == OwnedOperation::kPlanFollow) {
      follow_heartbeat_active_ = false;
      follow_session_id_.clear();
    }
  }
  const auto cancel_response =
    [this, lifetime = callback_lifetime_](const auto & response) {
      std::shared_lock lifetime_lock(lifetime->mutex);
      if (!lifetime->alive) {
        return;
      }
      observeCancelResponse(response);
    };
  try {
    if (follow) {
      follow_client_->async_cancel_goal(follow, cancel_response);
    } else if (execute_dig) {
      execute_dig_client_->async_cancel_goal(execute_dig, cancel_response);
    } else if (execute_dump) {
      execute_dump_client_->async_cancel_goal(execute_dump, cancel_response);
    } else if (excavation_cycle) {
      excavation_cycle_client_->async_cancel_goal(excavation_cycle, cancel_response);
    } else if (plan) {
      plan_client_->async_cancel_goal(plan, cancel_response);
    } else if (return_home) {
      return_home_client_->async_cancel_goal(return_home, cancel_response);
    } else if (hold_to_jog) {
      hold_to_jog_client_->async_cancel_goal(hold_to_jog, cancel_response);
    }
  } catch (const std::exception & error) {
    std::scoped_lock lock(mutex_);
    if (owned_operation_ != OwnedOperation::kIdle && cancel_requested_) {
      feedback_text_ = std::string("Cancel request error; waiting for Result: ") + error.what();
    }
  }
}

void ExcavationPanel::clearLogs()
{
  std::scoped_lock lock(mutex_);
  operator_logs_.clear();
  ++operator_log_revision_;
}

void ExcavationPanel::observeCancelResponse(
  const action_msgs::srv::CancelGoal::Response::SharedPtr & response)
{
  std::scoped_lock lock(mutex_);
  if (owned_operation_ == OwnedOperation::kIdle || !cancel_requested_) {
    return;
  }
  if (
    response && response->return_code == response->ERROR_NONE &&
    !response->goals_canceling.empty())
  {
    feedback_text_ = "Cancel accepted; waiting for terminal Result";
  } else {
    feedback_text_ = "Cancel not accepted; waiting for actual terminal Result";
  }
}

void ExcavationPanel::finishOperationLocked(const std::string & result_text)
{
  owned_operation_ = OwnedOperation::kIdle;
  cancel_requested_ = false;
  active_phase_.clear();
  plan_goal_handle_.reset();
  follow_goal_handle_.reset();
  execute_dig_goal_handle_.reset();
  execute_dump_goal_handle_.reset();
  excavation_cycle_goal_handle_.reset();
  return_home_goal_handle_.reset();
  hold_to_jog_goal_handle_.reset();
  jog_heartbeat_active_ = false;
  jog_session_id_.clear();
  follow_heartbeat_active_ = false;
  follow_session_id_.clear();
  operation_text_ = "Idle";
  result_text_ = result_text;
}

void ExcavationPanel::failOperationLocked(const std::string & result_text)
{
  finishOperationLocked(result_text);
}

PanelView ExcavationPanel::panelViewLocked(const rclcpp::Time & now) const
{
  RuntimeSnapshot runtime = runtime_;
  if (runtime.received && runtime_stamp_.nanoseconds() > 0) {
    const auto age = (now - runtime_stamp_).seconds();
    runtime.fresh = age >= 0.0 && age <= kStatusMaxAgeS;
  }

  const auto is_fresh = [&now](
    const airy_excavator_interfaces::msg::TargetSnapshot::SharedPtr & target,
    const rclcpp::Time & stamp) {
      if (!target || stamp.nanoseconds() <= 0) {
        return false;
      }
      const auto age = (now - stamp).seconds();
      return age >= 0.0 && age <= kTargetMaxAgeS;
    };
  OperatorResources resources;
  const bool plan_follow_ready =
    plan_client_ && follow_client_ && plan_client_->action_server_is_ready() &&
    follow_client_->action_server_is_ready();
  resources.dig_target_available =
    plan_follow_ready && is_fresh(dig_target_, dig_stamp_) &&
    dig_target_->target_kind == "dig" && dig_target_->mission_phase == "dig";
  resources.dump_target_available =
    plan_follow_ready && is_fresh(dump_target_, dump_stamp_) &&
    dump_target_->target_kind == "dump" && dump_target_->mission_phase == "dump";
  resources.home_pose_available =
    !home_pose_ids_.empty() && !home_pose_set_sha256_.empty() && return_home_client_ &&
    return_home_client_->action_server_is_ready();
  resources.execute_dig_available =
    execute_dig_client_ && execute_dig_client_->action_server_is_ready();
  resources.execute_dump_available =
    execute_dump_client_ && execute_dump_client_->action_server_is_ready();
  resources.full_mission_available =
    resources.dig_target_available && resources.dump_target_available &&
    resources.execute_dig_available && resources.execute_dump_available &&
    excavation_cycle_client_ && excavation_cycle_client_->action_server_is_ready();
  resources.manual_jog_available =
    hold_to_jog_client_ && hold_to_jog_client_->action_server_is_ready();
  return derive_panel_view(runtime, resources, owned_operation_);
}

void ExcavationPanel::refreshView()
{
  RuntimeSnapshot runtime;
  PanelView view;
  std::string operation_text;
  std::string feedback_text;
  std::string result_text;
  std::vector<std::string> pose_ids;
  std::vector<std::string> pose_statuses;
  std::size_t catalog_revision;
  std::vector<OperatorLogEntry> operator_logs;
  std::size_t operator_log_revision;
  {
    std::scoped_lock lock(mutex_);
    runtime = runtime_;
    const auto now = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    if (runtime.received && runtime_stamp_.nanoseconds() > 0) {
      const auto age = (now - runtime_stamp_).seconds();
      runtime.fresh = age >= 0.0 && age <= kStatusMaxAgeS;
    }
    view = panelViewLocked(now);
    operation_text = operation_text_;
    feedback_text = feedback_text_;
    result_text = result_text_;
    pose_ids = home_pose_ids_;
    pose_statuses = home_pose_statuses_;
    catalog_revision = home_catalog_revision_;
    operator_logs = operator_logs_;
    operator_log_revision = operator_log_revision_;
  }

  dig_button_->setEnabled(view.plan_follow_dig_enabled);
  dump_button_->setEnabled(view.plan_follow_dump_enabled);
  dig_button_->setText("Plan + Follow DIG");
  dump_button_->setText("Plan + Follow DUMP");
  return_home_button_->setEnabled(view.return_home_enabled);
  execute_dig_button_->setEnabled(view.execute_dig_enabled);
  execute_dump_button_->setEnabled(view.execute_dump_enabled);
  full_mission_button_->setEnabled(view.full_mission_enabled);
  refreshManualJogControls(view);
  cancel_button_->setEnabled(view.cancel_enabled);
  safety_label_->setText(QString::fromStdString(view.safety_text));
  const bool safety_ready =
    view.safety_text.rfind("LOCKED", 0) != 0 &&
    view.safety_text.size() >= 8 &&
    view.safety_text.compare(view.safety_text.size() - 8, 8, " / READY") == 0;
  safety_label_->setStyleSheet(
    safety_ready ?
    "font-weight: bold; color: #66dd88;" : "font-weight: bold; color: #ff6666;");
  runtime_label_->setText(QString::fromStdString(
      runtime.input_source + " / " + runtime.execution_mode + " / " +
      runtime.motion_backend + " / datagrams=" +
      std::to_string(runtime.action_datagrams) + " / gate=" +
      runtime.motion_gate_reason + " / fixed_actions=" +
      (runtime.fixed_actions_validated ? "field_validated" : "placeholder") +
      " / manual_jog=" + (runtime.manual_jog_ready ? "ready" : "locked")));
  operation_label_->setText(QString::fromStdString(operation_text));
  feedback_label_->setText(QString::fromStdString(feedback_text));
  result_label_->setText(QString::fromStdString(result_text));
  follow_status_label_->setText(QString::fromStdString(view.follow_status_text));
  const bool supervised_canary = runtime.follow_control_mode == "supervised_canary";
  follow_status_label_->setStyleSheet(
    supervised_canary ? "font-weight: bold; color: #ffcc66;" : "");

  if (catalog_revision != rendered_home_catalog_revision_) {
    const auto previous = home_pose_combo_->currentText();
    home_pose_combo_->clear();
    for (std::size_t index = 0; index < pose_ids.size(); ++index) {
      const auto label = pose_ids[index] + " [" + pose_statuses[index] + "]";
      home_pose_combo_->addItem(QString::fromStdString(label));
    }
    const auto previous_index = home_pose_combo_->findText(previous);
    if (previous_index >= 0) {
      home_pose_combo_->setCurrentIndex(previous_index);
    }
    rendered_home_catalog_revision_ = catalog_revision;
  }

  if (operator_log_revision != rendered_operator_log_revision_) {
    log_table_->setRowCount(static_cast<int>(operator_logs.size()));
    for (std::size_t index = 0; index < operator_logs.size(); ++index) {
      const auto & entry = operator_logs[index];
      auto * time_item = new QTableWidgetItem(format_log_stamp(entry.stamp_ns));
      auto * level_item = new QTableWidgetItem(severity_text(entry.severity));
      auto * count_item = new QTableWidgetItem(QString::number(entry.repeat_count));
      auto * module_item = new QTableWidgetItem(QString::fromStdString(entry.module));
      auto * message_item = new QTableWidgetItem(QString::fromStdString(entry.message));
      const auto color = severity_color(entry.severity);
      for (auto * item : {time_item, level_item, count_item, module_item, message_item}) {
        item->setForeground(color);
        item->setToolTip(
          format_log_stamp(entry.stamp_ns) + " [" +
          QString::fromStdString(entry.module) + "] " +
          QString::fromStdString(entry.message) +
          QString(" (repeated %1 times)").arg(entry.repeat_count));
      }
      log_table_->setItem(static_cast<int>(index), 0, time_item);
      log_table_->setItem(static_cast<int>(index), 1, level_item);
      log_table_->setItem(static_cast<int>(index), 2, count_item);
      log_table_->setItem(static_cast<int>(index), 3, module_item);
      log_table_->setItem(static_cast<int>(index), 4, message_item);
    }
    log_box_->setTitle(
      QString("Log History — Warnings / Errors (/rosout) — %1").arg(operator_logs.size()));
    if (!operator_logs.empty()) {
      log_table_->scrollToBottom();
    }
    rendered_operator_log_revision_ = operator_log_revision;
  }
  refreshJointTestControls(runtime);
  publishJointTestState(true);
}

}  // namespace airy_mission_panel

PLUGINLIB_EXPORT_CLASS(airy_mission_panel::ExcavationPanel, rviz_common::Panel)

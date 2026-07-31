#ifndef AIRY_MISSION_PANEL__EXCAVATION_PANEL_HPP_
#define AIRY_MISSION_PANEL__EXCAVATION_PANEL_HPP_

#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include <QWidget>

#include "action_msgs/srv/cancel_goal.hpp"
#include "airy_excavator_interfaces/action/follow.hpp"
#include "airy_excavator_interfaces/action/execute_dig.hpp"
#include "airy_excavator_interfaces/action/execute_dump.hpp"
#include "airy_excavator_interfaces/action/excavation_cycle.hpp"
#include "airy_excavator_interfaces/action/plan.hpp"
#include "airy_excavator_interfaces/action/return_home.hpp"
#include "airy_excavator_interfaces/msg/home_pose_catalog.hpp"
#include "airy_excavator_interfaces/msg/runtime_status.hpp"
#include "airy_excavator_interfaces/msg/target_snapshot.hpp"
#include "airy_excavator_interfaces/msg/trajectory_snapshot.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "airy_mission_panel/panel_state.hpp"

class QLabel;
class QPushButton;
class QCheckBox;
class QComboBox;
class QGroupBox;
class QTableWidget;
class QTabWidget;
class QSlider;
class QTimer;

namespace airy_mission_panel
{

class ExcavationPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ExcavationPanel(QWidget * parent = nullptr);
  ~ExcavationPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void startDig();
  void startDump();
  void startReturnHome();
  void startExecuteDig();
  void startExecuteDump();
  void startFullMission();
  void cancelOwnedOperation();
  void clearLogs();
  void refreshView();

private:
  struct CallbackLifetime
  {
    std::shared_mutex mutex;
    bool alive{true};
  };

  using Plan = airy_excavator_interfaces::action::Plan;
  using Follow = airy_excavator_interfaces::action::Follow;
  using ExecuteDig = airy_excavator_interfaces::action::ExecuteDig;
  using ExecuteDump = airy_excavator_interfaces::action::ExecuteDump;
  using ExcavationCycle = airy_excavator_interfaces::action::ExcavationCycle;
  using ReturnHome = airy_excavator_interfaces::action::ReturnHome;
  using PlanGoalHandle = rclcpp_action::ClientGoalHandle<Plan>;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Follow>;
  using ExecuteDigGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteDig>;
  using ExecuteDumpGoalHandle = rclcpp_action::ClientGoalHandle<ExecuteDump>;
  using ExcavationCycleGoalHandle = rclcpp_action::ClientGoalHandle<ExcavationCycle>;
  using ReturnHomeGoalHandle = rclcpp_action::ClientGoalHandle<ReturnHome>;

  void createRosInterfaces();
  void resetJointTests();
  void publishJointTestState(bool require_continuous);
  void refreshJointTestControls(const RuntimeSnapshot & runtime);
  PanelView panelViewLocked(const rclcpp::Time & now) const;
  void startPlanFollow(const std::string & phase);
  void sendFollow(const airy_excavator_interfaces::msg::TrajectorySnapshot & trajectory);
  void sendExecute(const std::string & phase);
  void observeCancelResponse(
    const action_msgs::srv::CancelGoal::Response::SharedPtr & response);
  void finishOperationLocked(const std::string & result_text);
  void failOperationLocked(const std::string & result_text);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<CallbackLifetime> callback_lifetime_{
    std::make_shared<CallbackLifetime>()};
  rclcpp_action::Client<Plan>::SharedPtr plan_client_;
  rclcpp_action::Client<Follow>::SharedPtr follow_client_;
  rclcpp_action::Client<ExecuteDig>::SharedPtr execute_dig_client_;
  rclcpp_action::Client<ExecuteDump>::SharedPtr execute_dump_client_;
  rclcpp_action::Client<ExcavationCycle>::SharedPtr excavation_cycle_client_;
  rclcpp_action::Client<ReturnHome>::SharedPtr return_home_client_;
  rclcpp::Subscription<airy_excavator_interfaces::msg::RuntimeStatus>::SharedPtr status_subscription_;
  rclcpp::Subscription<airy_excavator_interfaces::msg::TargetSnapshot>::SharedPtr dig_subscription_;
  rclcpp::Subscription<airy_excavator_interfaces::msg::TargetSnapshot>::SharedPtr dump_subscription_;
  rclcpp::Subscription<airy_excavator_interfaces::msg::HomePoseCatalog>::SharedPtr home_subscription_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_test_publisher_;

  mutable std::mutex mutex_;
  RuntimeSnapshot runtime_;
  rclcpp::Time runtime_stamp_{0, 0, RCL_ROS_TIME};
  airy_excavator_interfaces::msg::TargetSnapshot::SharedPtr dig_target_;
  airy_excavator_interfaces::msg::TargetSnapshot::SharedPtr dump_target_;
  rclcpp::Time dig_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time dump_stamp_{0, 0, RCL_ROS_TIME};
  std::string home_pose_set_sha256_;
  std::vector<std::string> home_pose_ids_;
  std::vector<std::string> home_pose_statuses_;
  std::size_t home_catalog_revision_{0};
  std::size_t rendered_home_catalog_revision_{0};
  std::vector<OperatorLogEntry> operator_logs_;
  std::size_t operator_log_revision_{0};
  std::size_t rendered_operator_log_revision_{0};
  OwnedOperation owned_operation_{OwnedOperation::kIdle};
  std::string active_phase_;
  std::string operation_text_{"Idle"};
  std::string feedback_text_{"-"};
  std::string result_text_{"-"};
  bool cancel_requested_{false};
  bool embedded_joint_tests_enabled_{false};
  std::uint64_t joint_test_publish_count_{0};
  PlanGoalHandle::SharedPtr plan_goal_handle_;
  FollowGoalHandle::SharedPtr follow_goal_handle_;
  ExecuteDigGoalHandle::SharedPtr execute_dig_goal_handle_;
  ExecuteDumpGoalHandle::SharedPtr execute_dump_goal_handle_;
  ExcavationCycleGoalHandle::SharedPtr excavation_cycle_goal_handle_;
  ReturnHomeGoalHandle::SharedPtr return_home_goal_handle_;

  QLabel * safety_label_;
  QLabel * runtime_label_;
  QLabel * operation_label_;
  QLabel * feedback_label_;
  QLabel * result_label_;
  QLabel * follow_status_label_;
  QPushButton * dig_button_;
  QPushButton * dump_button_;
  QPushButton * return_home_button_;
  QPushButton * cancel_button_;
  QPushButton * execute_dig_button_;
  QPushButton * execute_dump_button_;
  QPushButton * full_mission_button_;
  QTabWidget * tabs_;
  QGroupBox * log_box_;
  QTableWidget * log_table_;
  QPushButton * clear_log_button_;
  QLabel * joint_test_status_label_;
  QCheckBox * joint_test_continuous_checkbox_;
  std::array<QSlider *, kJointTestCount> joint_test_sliders_{};
  std::array<QLabel *, kJointTestCount> joint_test_value_labels_{};
  QPushButton * joint_test_publish_button_;
  QPushButton * joint_test_reset_button_;
  QComboBox * home_pose_combo_;
  QTimer * refresh_timer_;
};

}  // namespace airy_mission_panel

#endif  // AIRY_MISSION_PANEL__EXCAVATION_PANEL_HPP_

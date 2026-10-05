#ifndef ROBOT_CONTROLLER__ROBOT_CONTROLLER_NODE_HPP_
#define ROBOT_CONTROLLER__ROBOT_CONTROLLER_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/action/execute_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <robot_controller_msgs/msg/collision_object_map.hpp>
#include <robot_controller_msgs/srv/add_collision_objects.hpp>
#include <robot_controller_msgs/srv/apply_attached_collision_objects.hpp>
#include <robot_controller_msgs/srv/execute_joint_waypoints.hpp>
#include <robot_controller_msgs/srv/execute_joints.hpp>
#include <robot_controller_msgs/srv/execute_pose.hpp>
#include <robot_controller_msgs/srv/execute_waypoints.hpp>
#include <robot_controller_msgs/srv/get_collision_objects_from_scene.hpp>
#include <robot_controller_msgs/srv/get_joint_limits.hpp>
#include <robot_controller_msgs/srv/get_joint_states.hpp>
#include <robot_controller_msgs/srv/get_plan.hpp>
#include <robot_controller_msgs/srv/get_pose.hpp>
#include <robot_controller_msgs/srv/move_collision_objects.hpp>
#include <robot_controller_msgs/srv/push_pose_array.hpp>
#include <robot_controller_msgs/srv/remove_collision_objects.hpp>
#include <robot_controller_msgs/srv/robot_speed.hpp>

#include "robot_controller/move_group_interface.hpp"

namespace robot_controller
{

/**
 * Top-level ROS 2 node exposing a curated set of services for a robot arm.
 *
 * The node requires two-phase construction:
 *
 *     auto node = std::make_shared<RobotControllerNode>(options);
 *     node->initialize();
 *
 * The split exists because MoveGroupInterface::init() uses
 * shared_from_this() on the node, which is only valid once the node is
 * owned by a shared_ptr.
 */
class RobotControllerNode : public rclcpp::Node
{
public:
  explicit RobotControllerNode(const rclcpp::NodeOptions & options);
  ~RobotControllerNode() override = default;

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------
  /// Complete initialization: create MoveGroupInterface, callback groups,
  /// publishers, services, and action clients.  Must be called after the
  /// node is constructed with std::make_shared.
  bool initialize();

  // -------------------------------------------------------------------------
  // Public helpers (also used internally by service callbacks)
  // -------------------------------------------------------------------------

  /// Execute a sequence of Cartesian waypoints: plan a Cartesian path,
  /// apply time parameterization, then either execute synchronously or
  /// dispatch asynchronously.
  ///
  /// @param async_execute  true  -> send to execute_trajectory action and return
  ///                       false -> block on MoveGroupInterface::execute
  bool executeWaypoints(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double eef_step,
    double jump_threshold,
    double speed_percent,
    bool async_execute,
    moveit_msgs::msg::RobotTrajectory & out_trajectory,
    moveit_msgs::msg::MoveItErrorCodes & out_error_code,
    std::string & out_message);

  /// Dispatch an already-planned trajectory.
  bool executePlannedTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    bool async_execute,
    moveit_msgs::msg::MoveItErrorCodes & out_error_code,
    std::string & out_message);

  /// True while any execution (sync or async) is in progress.
  bool isExecuting() const;

  /// Result of the most recent async execution.  Before any async execution
  /// has finished, the value is MoveItErrorCodes::SUCCESS.
  moveit_msgs::msg::MoveItErrorCodes lastExecutionResult() const;

private:
  // -------------------------------------------------------------------------
  // Service type aliases
  // -------------------------------------------------------------------------
  using Trigger = std_srvs::srv::Trigger;

  using ExecuteJoints         = robot_controller_msgs::srv::ExecuteJoints;
  using ExecutePose           = robot_controller_msgs::srv::ExecutePose;
  using ExecuteWaypoints      = robot_controller_msgs::srv::ExecuteWaypoints;
  using ExecuteJointWaypoints = robot_controller_msgs::srv::ExecuteJointWaypoints;
  using GetPlan               = robot_controller_msgs::srv::GetPlan;

  using AddCollisionObjects           = robot_controller_msgs::srv::AddCollisionObjects;
  using RemoveCollisionObjects        = robot_controller_msgs::srv::RemoveCollisionObjects;
  using MoveCollisionObjects          = robot_controller_msgs::srv::MoveCollisionObjects;
  using ApplyAttachedCollisionObjects = robot_controller_msgs::srv::ApplyAttachedCollisionObjects;
  using GetCollisionObjectsFromScene  = robot_controller_msgs::srv::GetCollisionObjectsFromScene;

  using GetJointStates = robot_controller_msgs::srv::GetJointStates;
  using GetJointLimits = robot_controller_msgs::srv::GetJointLimits;
  using GetPose        = robot_controller_msgs::srv::GetPose;
  using PushPoseArray  = robot_controller_msgs::srv::PushPoseArray;
  using RobotSpeed     = robot_controller_msgs::srv::RobotSpeed;

  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;

  // -------------------------------------------------------------------------
  // Setup helpers
  // -------------------------------------------------------------------------
  bool loadParameters();
  void createCallbackGroups();
  void createPublishers();
  void createServices();
  void createActionClients();

  // -------------------------------------------------------------------------
  // Internal helpers
  // -------------------------------------------------------------------------

  /// Convert a numeric speed request (percent, 0.0 = use default) into a
  /// scaling factor in (0, 1].  Returns false and sets out_message on error.
  bool parseSpeed(
    double speed_percent,
    double & out_scaling,
    std::string & out_message) const;

  /// Send a trajectory to the execute_trajectory action server.
  /// Returns true if the goal was accepted.  Sets out_message in all cases.
  bool sendAsyncTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    std::string & out_message);

  // -------------------------------------------------------------------------
  // Service callbacks
  // -------------------------------------------------------------------------
  void cbTesting(
    std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response>);

  void cbStopExecution(
    std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response>);

  void cbExecuteJoints(
    std::shared_ptr<ExecuteJoints::Request>,
    std::shared_ptr<ExecuteJoints::Response>);

  void cbExecutePose(
    std::shared_ptr<ExecutePose::Request>,
    std::shared_ptr<ExecutePose::Response>);

  void cbExecuteWaypoints(
    std::shared_ptr<ExecuteWaypoints::Request>,
    std::shared_ptr<ExecuteWaypoints::Response>);

  void cbExecuteJointWaypoints(
    std::shared_ptr<ExecuteJointWaypoints::Request>,
    std::shared_ptr<ExecuteJointWaypoints::Response>);

  void cbGetPlan(
    std::shared_ptr<GetPlan::Request>,
    std::shared_ptr<GetPlan::Response>);

  void cbAddCollisionObject(
    std::shared_ptr<AddCollisionObjects::Request>,
    std::shared_ptr<AddCollisionObjects::Response>);

  void cbRemoveCollisionObject(
    std::shared_ptr<RemoveCollisionObjects::Request>,
    std::shared_ptr<RemoveCollisionObjects::Response>);

  void cbApplyAttachedCollisionObject(
    std::shared_ptr<ApplyAttachedCollisionObjects::Request>,
    std::shared_ptr<ApplyAttachedCollisionObjects::Response>);

  void cbMoveCollisionObject(
    std::shared_ptr<MoveCollisionObjects::Request>,
    std::shared_ptr<MoveCollisionObjects::Response>);

  void cbGetCollisionObjectsFromScene(
    std::shared_ptr<GetCollisionObjectsFromScene::Request>,
    std::shared_ptr<GetCollisionObjectsFromScene::Response>);

  void cbRobotSpeed(
    std::shared_ptr<RobotSpeed::Request>,
    std::shared_ptr<RobotSpeed::Response>);

  void cbGetPose(
    std::shared_ptr<GetPose::Request>,
    std::shared_ptr<GetPose::Response>);

  void cbGetJointStates(
    std::shared_ptr<GetJointStates::Request>,
    std::shared_ptr<GetJointStates::Response>);

  void cbGetJointLimits(
    std::shared_ptr<GetJointLimits::Request>,
    std::shared_ptr<GetJointLimits::Response>);

  void cbPushPoseArray(
    std::shared_ptr<PushPoseArray::Request>,
    std::shared_ptr<PushPoseArray::Response>);

  void cbClearPoseArray(
    std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response>);

  // -------------------------------------------------------------------------
  // Configuration
  // -------------------------------------------------------------------------
  std::string move_group_ns_;
  std::string group_name_;
  std::string eef_name_;
  std::string ref_frame_;
  double default_eef_step_{0.01};
  double default_jump_threshold_{5.0};
  double default_speed_percent_{50.0};
  bool   use_bspline_{false};
  double bspline_step_{0.1};

  // -------------------------------------------------------------------------
  // MoveIt2 wrapper
  // -------------------------------------------------------------------------
  std::shared_ptr<MoveGroupInterface> move_group_;

  // -------------------------------------------------------------------------
  // Pushed waypoint buffer
  // -------------------------------------------------------------------------
  mutable std::mutex pushed_waypoints_mtx_;
  std::vector<geometry_msgs::msg::Pose> pushed_waypoints_;

  // -------------------------------------------------------------------------
  // Callback groups
  // -------------------------------------------------------------------------
  rclcpp::CallbackGroup::SharedPtr srv_general_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_exec_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_stop_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_collision_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_col_query_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_cbg_;

  // -------------------------------------------------------------------------
  // Publishers
  // -------------------------------------------------------------------------
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;

  // -------------------------------------------------------------------------
  // Action client for async execution
  // -------------------------------------------------------------------------
  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exec_traj_action_cli_;

  // -------------------------------------------------------------------------
  // Execution state
  // -------------------------------------------------------------------------
  std::atomic<bool> async_exec_in_progress_{false};
  std::atomic<bool> sync_exec_in_progress_{false};
  mutable std::mutex last_async_result_mtx_;
  moveit_msgs::msg::MoveItErrorCodes last_async_result_;

  // -------------------------------------------------------------------------
  // Service servers
  // -------------------------------------------------------------------------
  rclcpp::Service<Trigger>::SharedPtr testing_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_exec_srv_;

  rclcpp::Service<ExecuteJoints>::SharedPtr         exec_joints_srv_;
  rclcpp::Service<ExecutePose>::SharedPtr           exec_pose_srv_;
  rclcpp::Service<ExecuteWaypoints>::SharedPtr      exec_waypoints_srv_;
  rclcpp::Service<ExecuteJointWaypoints>::SharedPtr exec_joint_waypoints_srv_;
  rclcpp::Service<GetPlan>::SharedPtr               get_plan_srv_;

  rclcpp::Service<AddCollisionObjects>::SharedPtr           add_collision_obj_srv_;
  rclcpp::Service<RemoveCollisionObjects>::SharedPtr        remove_collision_obj_srv_;
  rclcpp::Service<ApplyAttachedCollisionObjects>::SharedPtr apply_attached_collision_obj_srv_;
  rclcpp::Service<MoveCollisionObjects>::SharedPtr          move_collision_obj_srv_;
  rclcpp::Service<GetCollisionObjectsFromScene>::SharedPtr  get_collision_obj_from_scene_srv_;

  rclcpp::Service<RobotSpeed>::SharedPtr    robot_speed_srv_;
  rclcpp::Service<GetPose>::SharedPtr       get_pose_srv_;
  rclcpp::Service<GetJointStates>::SharedPtr get_joint_states_srv_;
  rclcpp::Service<GetJointLimits>::SharedPtr get_joint_limits_srv_;
  rclcpp::Service<PushPoseArray>::SharedPtr push_pose_arr_srv_;
  rclcpp::Service<Trigger>::SharedPtr       clear_pose_arr_srv_;
};

}  // namespace robot_controller

#endif  // ROBOT_CONTROLLER__ROBOT_CONTROLLER_NODE_HPP_
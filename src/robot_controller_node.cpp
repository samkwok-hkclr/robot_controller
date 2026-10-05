#include "robot_controller/robot_controller_node.hpp"

#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot_controller
{

using std::placeholders::_1;
using std::placeholders::_2;

// ---------------------------------------------------------------------------
RobotControllerNode::RobotControllerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("robot_controller", options)
{
  // Only parameter loading here; everything that needs shared_from_this()
  // is deferred to initialize().
  if (!loadParameters()) {
    throw std::runtime_error("Failed to load parameters");
  }

  last_async_result_.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  RCLCPP_INFO(get_logger(), "RobotControllerNode constructed. Ready to initialize.");
}

// ---------------------------------------------------------------------------
bool RobotControllerNode::loadParameters()
{
  auto declare_or_get_string =
    [this](const std::string & name, const std::string & def) -> std::string {
      if (has_parameter(name)) {
        return get_parameter(name).as_string();
      }
      return declare_parameter<std::string>(name, def);
    };

  auto declare_or_get_double =
    [this](const std::string & name, double def) -> double {
      if (has_parameter(name)) {
        return get_parameter(name).as_double();
      }
      return declare_parameter<double>(name, def);
    };

  auto declare_or_get_bool =
    [this](const std::string & name, bool def) -> bool {
      if (has_parameter(name)) {
        return get_parameter(name).as_bool();
      }
      return declare_parameter<bool>(name, def);
    };

  move_group_ns_           = declare_or_get_string("move_group_namespace", "");
  group_name_              = declare_or_get_string("group_name", "");
  eef_name_                = declare_or_get_string("eef_name", "");
  ref_frame_               = declare_or_get_string("ref_frame", "");
  default_eef_step_        = declare_or_get_double("default_eef_step", 0.01);
  default_jump_threshold_  = declare_or_get_double("default_jump_threshold", 5.0);
  default_speed_percent_   = declare_or_get_double("default_speed_percent", 50.0);
  use_bspline_             = declare_or_get_bool("use_bspline", false);
  bspline_step_            = declare_or_get_double("bspline_step", 0.1);

  if (group_name_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'group_name' is required");
    return false;
  }
  if (default_eef_step_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "default_eef_step <= 0; using 0.01");
    default_eef_step_ = 0.01;
  }
  if (default_speed_percent_ <= 0.0 || default_speed_percent_ > 100.0) {
    RCLCPP_WARN(get_logger(), "default_speed_percent out of range; using 50.0");
    default_speed_percent_ = 50.0;
  }
  return true;
}

// ---------------------------------------------------------------------------
bool RobotControllerNode::initialize()
{
  // Create the MoveGroupInterface wrapper.  Its init() calls
  // shared_from_this() on this node, so it must run after the node is owned
  // by a shared_ptr (i.e. after std::make_shared returns).
  move_group_ = std::make_shared<MoveGroupInterface>(*this, group_name_);

  if (!move_group_->init(move_group_ns_, group_name_, eef_name_, ref_frame_)) {
    RCLCPP_FATAL(get_logger(), "Failed to initialize MoveGroupInterface");
    return false;
  }

  move_group_->setUseBspline(use_bspline_, bspline_step_);
  move_group_->setDefaultSpeedScaling(1.0, 0.8);

  createCallbackGroups();
  createPublishers();
  createServices();
  createActionClients();

  RCLCPP_INFO(get_logger(), "RobotControllerNode is fully initialized and up");
  return true;
}

// ---------------------------------------------------------------------------
void RobotControllerNode::createCallbackGroups()
{
  srv_general_cbg_   = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_exec_cbg_      = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_stop_cbg_      = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  srv_collision_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_col_query_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cbg_        = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

// ---------------------------------------------------------------------------
void RobotControllerNode::createPublishers()
{
  speed_pub_ = create_publisher<std_msgs::msg::Float32>("robot_speed", rclcpp::QoS(10));
}

// ---------------------------------------------------------------------------
void RobotControllerNode::createActionClients()
{
  // Client for MoveIt2's execute_trajectory action server.
  exec_traj_action_cli_ =
    rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
      get_node_base_interface(),
      get_node_graph_interface(),
      get_node_logging_interface(),
      get_node_waitables_interface(),
      "/execute_trajectory",
      action_cbg_);
}

// ---------------------------------------------------------------------------
void RobotControllerNode::createServices()
{
  // Execution services (mutually exclusive with each other, not with stop).
  exec_joints_srv_ = create_service<ExecuteJoints>(
    "execute_joints",
    std::bind(&RobotControllerNode::cbExecuteJoints, this, _1, _2),
    rmw_qos_profile_services_default, srv_exec_cbg_);

  exec_pose_srv_ = create_service<ExecutePose>(
    "execute_pose",
    std::bind(&RobotControllerNode::cbExecutePose, this, _1, _2),
    rmw_qos_profile_services_default, srv_exec_cbg_);

  exec_waypoints_srv_ = create_service<ExecuteWaypoints>(
    "execute_waypoints",
    std::bind(&RobotControllerNode::cbExecuteWaypoints, this, _1, _2),
    rmw_qos_profile_services_default, srv_exec_cbg_);

  exec_joint_waypoints_srv_ = create_service<ExecuteJointWaypoints>(
    "execute_joint_waypoints",
    std::bind(&RobotControllerNode::cbExecuteJointWaypoints, this, _1, _2),
    rmw_qos_profile_services_default, srv_exec_cbg_);

  // Stop service — reentrant so it can preempt an in-flight execution.
  stop_exec_srv_ = create_service<Trigger>(
    "stop_execution",
    std::bind(&RobotControllerNode::cbStopExecution, this, _1, _2),
    rmw_qos_profile_services_default, srv_stop_cbg_);

  // Planning-only service.
  get_plan_srv_ = create_service<GetPlan>(
    "get_plan",
    std::bind(&RobotControllerNode::cbGetPlan, this, _1, _2),
    rmw_qos_profile_services_default, srv_exec_cbg_);

  // Collision mutations.
  add_collision_obj_srv_ = create_service<AddCollisionObjects>(
    "add_collision_objects",
    std::bind(&RobotControllerNode::cbAddCollisionObject, this, _1, _2),
    rmw_qos_profile_services_default, srv_collision_cbg_);

  remove_collision_obj_srv_ = create_service<RemoveCollisionObjects>(
    "remove_collision_objects",
    std::bind(&RobotControllerNode::cbRemoveCollisionObject, this, _1, _2),
    rmw_qos_profile_services_default, srv_collision_cbg_);

  apply_attached_collision_obj_srv_ = create_service<ApplyAttachedCollisionObjects>(
    "apply_attached_collision_objects",
    std::bind(&RobotControllerNode::cbApplyAttachedCollisionObject, this, _1, _2),
    rmw_qos_profile_services_default, srv_collision_cbg_);

  move_collision_obj_srv_ = create_service<MoveCollisionObjects>(
    "move_collision_objects",
    std::bind(&RobotControllerNode::cbMoveCollisionObject, this, _1, _2),
    rmw_qos_profile_services_default, srv_collision_cbg_);

  get_collision_obj_from_scene_srv_ = create_service<GetCollisionObjectsFromScene>(
    "get_collision_objects_from_scene",
    std::bind(&RobotControllerNode::cbGetCollisionObjectsFromScene, this, _1, _2),
    rmw_qos_profile_services_default, srv_col_query_cbg_);

  // Queries.
  testing_srv_ = create_service<Trigger>(
    "testing",
    std::bind(&RobotControllerNode::cbTesting, this, _1, _2),
    rmw_qos_profile_services_default, srv_general_cbg_);

  robot_speed_srv_ = create_service<RobotSpeed>(
    "robot_speed",
    std::bind(&RobotControllerNode::cbRobotSpeed, this, _1, _2),
    rmw_qos_profile_services_default, srv_general_cbg_);

  get_pose_srv_ = create_service<GetPose>(
    "get_pose",
    std::bind(&RobotControllerNode::cbGetPose, this, _1, _2),
    rmw_qos_profile_services_default, srv_general_cbg_);

  get_joint_states_srv_ = create_service<GetJointStates>(
    "get_current_joint_states",
    std::bind(&RobotControllerNode::cbGetJointStates, this, _1, _2),
    rmw_qos_profile_services_default, srv_general_cbg_);

  get_joint_limits_srv_ = create_service<GetJointLimits>(
    "get_joint_limits",
    std::bind(&RobotControllerNode::cbGetJointLimits, this, _1, _2),
    rmw_qos_profile_services_default, srv_general_cbg_);

  push_pose_arr_srv_ = create_service<PushPoseArray>(
    "push_pose_array",
    std::bind(&RobotControllerNode::cbPushPoseArray, this, _1, _2),
    rmw_qos_profile_services_default, srv_exec_cbg_);

  clear_pose_arr_srv_ = create_service<Trigger>(
    "clear_pose_array",
    std::bind(&RobotControllerNode::cbClearPoseArray, this, _1, _2),
    rmw_qos_profile_services_default, srv_exec_cbg_);
}

// ---------------------------------------------------------------------------
bool RobotControllerNode::parseSpeed(
  double speed_percent, double & out_scaling, std::string & out_message) const
{
  double percent = speed_percent;
  if (percent <= 0.0) {
    percent = default_speed_percent_;
  }
  if (percent <= 0.0 || percent > 100.0) {
    out_message = "speed must be in (0, 100]";
    return false;
  }
  out_scaling = percent / 100.0;
  return true;
}

// ---------------------------------------------------------------------------
bool RobotControllerNode::isExecuting() const
{
  return async_exec_in_progress_.load() || sync_exec_in_progress_.load();
}

// ---------------------------------------------------------------------------
moveit_msgs::msg::MoveItErrorCodes RobotControllerNode::lastExecutionResult() const
{
  std::lock_guard<std::mutex> lock(last_async_result_mtx_);
  return last_async_result_;
}

// ---------------------------------------------------------------------------
bool RobotControllerNode::sendAsyncTrajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  std::string & out_message)
{
  if (sync_exec_in_progress_.load()) {
    out_message = "A synchronous execution is already running";
    return false;
  }
  if (async_exec_in_progress_.load()) {
    out_message = "Another async execution is already in progress";
    return false;
  }
  if (!exec_traj_action_cli_) {
    out_message = "Async action client is not initialized";
    return false;
  }
  if (!exec_traj_action_cli_->action_server_is_ready()) {
    out_message = "execute_trajectory action server is not available";
    return false;
  }

  moveit_msgs::action::ExecuteTrajectory::Goal goal;
  goal.trajectory = trajectory;

  rclcpp_action::Client<moveit_msgs::action::ExecuteTrajectory>::SendGoalOptions options;

  options.feedback_callback =
    [this](
      rclcpp_action::ClientGoalHandle<moveit_msgs::action::ExecuteTrajectory>::SharedPtr,
      const std::shared_ptr<const moveit_msgs::action::ExecuteTrajectory::Feedback> fb)
    {
      if (fb) {
        RCLCPP_DEBUG(get_logger(), "Async execution feedback: %s", fb->state.c_str());
      }
    };

  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<
             moveit_msgs::action::ExecuteTrajectory>::WrappedResult & result)
    {
      moveit_msgs::msg::MoveItErrorCodes code;
      if (result.result) {
        code = result.result->error_code;
      } else {
        code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      }
      {
        std::lock_guard<std::mutex> lock(last_async_result_mtx_);
        last_async_result_ = code;
      }
      async_exec_in_progress_.store(false);
      RCLCPP_INFO(
        get_logger(), "Async execution finished with MoveIt error code %d", code.val);
    };

  async_exec_in_progress_.store(true);

  auto goal_handle_future = exec_traj_action_cli_->async_send_goal(goal, options);

  // Wait briefly for goal acceptance so we can report failure honestly.
  if (goal_handle_future.wait_for(std::chrono::milliseconds(500)) !=
      std::future_status::ready)
  {
    async_exec_in_progress_.store(false);
    out_message = "Timeout waiting for execute_trajectory goal response";
    return false;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    async_exec_in_progress_.store(false);
    out_message = "Async execution goal was rejected by the action server";
    return false;
  }

  out_message = "Async execution started";
  return true;
}

// ---------------------------------------------------------------------------
bool RobotControllerNode::executePlannedTrajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  bool async_execute,
  moveit_msgs::msg::MoveItErrorCodes & out_error_code,
  std::string & out_message)
{
  if (async_execute) {
    // Async path: hand the trajectory off and return as soon as the goal
    // is accepted by the execute_trajectory action server.
    if (!sendAsyncTrajectory(trajectory, out_message)) {
      out_error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      return false;
    }
    out_error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return true;
  }

  // Synchronous path: block until the motion completes.
  if (async_exec_in_progress_.load()) {
    out_message = "Cannot execute synchronously: an async execution is in progress";
    out_error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }
  if (sync_exec_in_progress_.exchange(true)) {
    out_message = "Another synchronous execution is already in progress";
    out_error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  RCLCPP_INFO(get_logger(), "Starting synchronous execution...");
  out_error_code = move_group_->execute(trajectory);
  sync_exec_in_progress_.store(false);

  if (out_error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    out_message = "Synchronous execution failed (code " +
      std::to_string(out_error_code.val) + ")";
    return false;
  }
  out_message = "Synchronous execution succeeded";
  return true;
}

// ---------------------------------------------------------------------------
bool RobotControllerNode::executeWaypoints(
  const std::vector<geometry_msgs::msg::Pose> & input_waypoints,
  double eef_step,
  double jump_threshold,
  double speed_percent,
  bool async_execute,
  moveit_msgs::msg::RobotTrajectory & out_trajectory,
  moveit_msgs::msg::MoveItErrorCodes & out_error_code,
  std::string & out_message)
{
  if (input_waypoints.empty()) {
    out_message = "Input waypoints is empty";
    out_error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  double scaling = 1.0;
  if (!parseSpeed(speed_percent, scaling, out_message)) {
    return false;
  }

  // Merge any pushed waypoints (from push_pose_array) with the incoming ones.
  std::vector<geometry_msgs::msg::Pose> merged;
  {
    std::lock_guard<std::mutex> lock(pushed_waypoints_mtx_);
    merged.reserve(pushed_waypoints_.size() + input_waypoints.size());
    merged.insert(merged.end(), pushed_waypoints_.begin(), pushed_waypoints_.end());
    pushed_waypoints_.clear();
  }
  merged.insert(merged.end(), input_waypoints.begin(), input_waypoints.end());

  // Normalize quaternions in place.
  for (auto & pose : merged) {
    auto & q = pose.orientation;
    const double n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (n2 > 0.0 && std::abs(n2 - 1.0) > 1e-6) {
      const double inv = 1.0 / std::sqrt(n2);
      q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
    }
  }

  if (eef_step <= 0.0) eef_step = default_eef_step_;
  if (jump_threshold <= 0.0) jump_threshold = default_jump_threshold_;

  moveit_msgs::msg::MoveItErrorCodes plan_err;
  const double fraction = move_group_->planCartesianPath(
    merged, eef_step, jump_threshold, out_trajectory,
    /*avoid_collisions=*/true, &plan_err);

  if (fraction < 1.0) {
    out_message = "Cartesian path planning reached only " +
      std::to_string(static_cast<int>(fraction * 100.0)) + "%";
    out_error_code = plan_err;
    return false;
  }

  if (!move_group_->applyTimeParameterization(out_trajectory, scaling, scaling)) {
    out_message = "Time parameterization failed";
    out_error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  return executePlannedTrajectory(out_trajectory, async_execute, out_error_code, out_message);
}

// ---------------------------------------------------------------------------
void RobotControllerNode::cbTesting(
  const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
{
  response->success = true;
  response->message = "testing ok";
}

}  // namespace robot_controller
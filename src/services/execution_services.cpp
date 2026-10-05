#include "robot_controller/robot_controller_node.hpp"

#include <sstream>
#include <string>
#include <unordered_map>

namespace robot_controller
{

// ---------------------------------------------------------------------------
void RobotControllerNode::cbExecuteJoints(
  const std::shared_ptr<ExecuteJoints::Request> request,
  std::shared_ptr<ExecuteJoints::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(current_source_mtx_);
    current_source_ = "execute_joints";
  }

  response->success = false;

  if (request->joint_names.size() != request->joint_positions.size()) {
    response->message = "joint_names and joint_positions size mismatch";
    return;
  }
  if (request->joint_positions.empty()) {
    response->message = "joint_positions is empty";
    return;
  }

  // Limits check.
  auto limits_opt = move_group_->getJointLimits();
  if (limits_opt) {
    std::unordered_map<std::string, moveit_msgs::msg::JointLimits> limit_map;
    for (const auto & lim : *limits_opt) limit_map[lim.joint_name] = lim;

    for (std::size_t i = 0; i < request->joint_names.size(); ++i) {
      const auto & name = request->joint_names[i];
      const double pos = request->joint_positions[i];
      auto it = limit_map.find(name);
      if (it == limit_map.end()) {
        RCLCPP_WARN(get_logger(), "No limits for joint '%s'", name.c_str());
        continue;
      }
      const auto & lim = it->second;
      if (lim.has_position_limits &&
          (pos < lim.min_position || pos > lim.max_position))
      {
        std::ostringstream ss;
        ss << "Joint '" << name << "' out of limits: " << pos
           << " not in [" << lim.min_position << ", " << lim.max_position << "]";
        response->message = ss.str();
        return;
      }
    }
  }

  moveit_msgs::msg::MoveItErrorCodes plan_err;
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (!move_group_->planJointTarget(request->joint_positions, trajectory, plan_err)) {
    response->message = "Joint planning failed (code " + std::to_string(plan_err.val) + ")";
    response->error_code = plan_err;
    return;
  }

  double scaling = 1.0;
  if (!parseSpeed(request->speed, scaling, response->message)) return;
  if (!move_group_->applyTimeParameterization(trajectory, scaling, scaling)) {
    response->message = "Time parameterization failed";
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  response->trajectory = std::move(trajectory);
  response->success = executePlannedTrajectory(
    response->trajectory, request->async_execute,
    response->error_code, response->message);
}

// ---------------------------------------------------------------------------
void RobotControllerNode::cbExecutePose(
  const std::shared_ptr<ExecutePose::Request> request,
  std::shared_ptr<ExecutePose::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(current_source_mtx_);
    current_source_ = "execute_pose";
  }

  response->success = false;

  moveit_msgs::msg::MoveItErrorCodes plan_err;
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (!move_group_->planPoseTarget(request->target_pose, trajectory, plan_err)) {
    response->message = "Pose planning failed (code " + std::to_string(plan_err.val) + ")";
    response->error_code = plan_err;
    return;
  }

  double scaling = 1.0;
  if (!parseSpeed(request->speed, scaling, response->message)) return;
  if (!move_group_->applyTimeParameterization(trajectory, scaling, scaling)) {
    response->message = "Time parameterization failed";
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  response->trajectory = std::move(trajectory);
  response->success = executePlannedTrajectory(
    response->trajectory, request->async_execute,
    response->error_code, response->message);
}

// ---------------------------------------------------------------------------
void RobotControllerNode::cbExecuteWaypoints(
  const std::shared_ptr<ExecuteWaypoints::Request> request,
  std::shared_ptr<ExecuteWaypoints::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(current_source_mtx_);
    current_source_ = "execute_waypoints";
  }

  response->success = executeWaypoints(
    request->waypoints,
    request->eef_step,
    request->jump_threshold,
    request->speed,
    request->async_execute,
    response->trajectory,
    response->error_code,
    response->message);
}

// ---------------------------------------------------------------------------
void RobotControllerNode::cbExecuteJointWaypoints(
  const std::shared_ptr<ExecuteJointWaypoints::Request> request,
  std::shared_ptr<ExecuteJointWaypoints::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(current_source_mtx_);
    current_source_ = "execute_joint_waypoints";
  }

  response->success = false;

  moveit_msgs::msg::MoveItErrorCodes plan_err;
  moveit_msgs::msg::RobotTrajectory trajectory;
  if (!move_group_->planJointWaypointsFromPoses(request->waypoints, trajectory, plan_err)) {
    response->message = "Joint-waypoint planning failed (code " +
      std::to_string(plan_err.val) + ")";
    response->error_code = plan_err;
    return;
  }

  double scaling = 1.0;
  if (!parseSpeed(request->speed, scaling, response->message)) return;
  if (!move_group_->applyTimeParameterization(trajectory, scaling, scaling)) {
    response->message = "Time parameterization failed";
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  response->trajectory = std::move(trajectory);
  response->success = executePlannedTrajectory(
    response->trajectory, request->async_execute,
    response->error_code, response->message);
}

// ---------------------------------------------------------------------------
void RobotControllerNode::cbStopExecution(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  move_group_->stop();
  response->success = true;
  response->message = "stop requested";
}

}  // namespace robot_controller
#include "robot_controller/robot_controller_node.hpp"

#include <sstream>
#include <string>
#include <vector>

namespace robot_controller
{

void RobotControllerNode::cbGetPlan(
  const std::shared_ptr<GetPlan::Request> request,
  std::shared_ptr<GetPlan::Response> response)
{
  response->success = false;
  response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes plan_err;
  double scaling = 1.0;

  switch (request->target_type) {
    // ---------------------------------------------------------------------
    case GetPlan::Request::TARGET_JOINTS: {
      if (request->joint_positions.empty()) {
        response->message = "joint_positions is empty";
        return;
      }
      if (!move_group_->planJointTarget(request->joint_positions, trajectory, plan_err)) {
        response->message = "Joint planning failed (code " +
          std::to_string(plan_err.val) + ")";
        response->error_code = plan_err;
        return;
      }
      break;
    }

    // ---------------------------------------------------------------------
    case GetPlan::Request::TARGET_POSE: {
      if (!move_group_->planPoseTarget(request->target_pose, trajectory, plan_err)) {
        response->message = "Pose planning failed (code " +
          std::to_string(plan_err.val) + ")";
        response->error_code = plan_err;
        return;
      }
      break;
    }

    // ---------------------------------------------------------------------
    case GetPlan::Request::TARGET_JOINT_WAYPOINTS: {
      if (request->joint_waypoints.empty()) {
        response->message = "joint_waypoints is empty";
        return;
      }

      // Unwrap the JointWaypoint[] message into std::vector<std::vector<double>>.
      std::vector<std::vector<double>> joint_waypoints;
      joint_waypoints.reserve(request->joint_waypoints.size());
      for (const auto & w : request->joint_waypoints) {
        if (w.positions.empty()) {
          response->message = "A JointWaypoint entry has empty positions";
          return;
        }
        joint_waypoints.push_back(w.positions);
      }

      if (!move_group_->planJointWaypoints(joint_waypoints, trajectory, plan_err)) {
        response->message = "Joint-waypoint planning failed (code " +
          std::to_string(plan_err.val) + ")";
        response->error_code = plan_err;
        return;
      }
      break;
    }

    // ---------------------------------------------------------------------
    case GetPlan::Request::TARGET_POSE_WAYPOINTS: {
      if (request->pose_waypoints.empty()) {
        response->message = "pose_waypoints is empty";
        return;
      }

      const double eef_step = request->eef_step > 0.0
        ? request->eef_step
        : default_eef_step_;
      const double jump_threshold = request->jump_threshold > 0.0
        ? request->jump_threshold
        : default_jump_threshold_;

      moveit_msgs::msg::MoveItErrorCodes path_err;
      const double fraction = move_group_->planCartesianPath(
        request->pose_waypoints,
        eef_step,
        jump_threshold,
        trajectory,
        /*avoid_collisions=*/true,
        &path_err);

      if (fraction < 1.0) {
        std::ostringstream ss;
        ss << "Cartesian path planning reached "
           << static_cast<int>(fraction * 100.0) << "%";
        response->message = ss.str();
        response->error_code = path_err;
        return;
      }
      break;
    }

    // ---------------------------------------------------------------------
    default:
      response->message = "Unknown target_type: " + std::to_string(request->target_type);
      return;
  }

  // Common tail: speed parsing + time parameterization.
  if (!parseSpeed(request->speed, scaling, response->message)) {
    return;
  }

  if (!move_group_->applyTimeParameterization(trajectory, scaling, scaling)) {
    response->message = "Time parameterization failed";
    response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  response->success = true;
  response->message = "Plan computed";
  response->trajectory = std::move(trajectory);
  response->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
}

}  // namespace robot_controller
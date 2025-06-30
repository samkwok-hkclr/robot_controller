#include "robot_controller/robot_ctlr_node.hpp"

void RobotControllerNode::exec_joints_cb(
  const std::shared_ptr<ExecuteJoints::Request> request, 
  std::shared_ptr<ExecuteJoints::Response> response)
{
  std::string ret_msg;
  bool success = exec_pushed_waypoints(&ret_msg);

  if (!success)
  {
    response->message = ret_msg;
    response->success = false;
    return;
  }

  if (move_group_->execute_joints(request->joints) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    std::string err_msg = "execute_joints failed !!!!!!";

    response->message = err_msg;
    response->success = false;

    RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Joints Execution finished!");
  response->success = true;
}

void RobotControllerNode::exec_pose_cb(
  const std::shared_ptr<ExecutePose::Request> request, 
  std::shared_ptr<ExecutePose::Response> response)
{
  std::string ret_msg;
  bool success = exec_pushed_waypoints(&ret_msg);

  if (!success)
  {
    response->message = ret_msg;
    response->success = false;
    return;
  }

  if (move_group_->execute_pose(request->pose) != moveit::core::MoveItErrorCode::SUCCESS)
  {
    std::string err_msg = "execute_pose failed !!!!!!";

    response->message = err_msg;
    response->success = false;

    RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Pose Execution finished!");
  response->success = true;
}

void RobotControllerNode::exec_joint_waypoints_cb(
  const std::shared_ptr<ExecuteJointWaypoints::Request> request, 
  std::shared_ptr<ExecuteJointWaypoints::Response> response)
{
  std::string ret_msg;
  bool success = exec_pushed_waypoints(&ret_msg);

  if (!success)
  {
    response->message = ret_msg;
    response->success = false;
    return;
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  success = move_group_->compute_joint_path(request->waypoints, trajectory);
  if (!success)
  {
    std::string err_msg = "compute_joint_path failed !!!!!!";

    response->message = err_msg;
    response->success = false;

    RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
    return;
  }

  moveit::core::MoveItErrorCode code = move_group_->execute(trajectory);
  if (code != moveit::core::MoveItErrorCode::SUCCESS)
  {
    std::string err_msg = "Execute failed with error: " + std::to_string(code.val);

    response->message = err_msg;
    response->success = false;

    RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Joint Waypoints Execution finished!");
  response->success = true;
}

void RobotControllerNode::exec_waypoints_cb(
  const std::shared_ptr<ExecuteWaypoints::Request> request, 
  std::shared_ptr<ExecuteWaypoints::Response> response)
{
  response->success = exec_waypoints(
    request->waypoints, 
    request->eef_step == 0.0 ? eef_step_.load() : request->eef_step,
    request->jump_threshold == 0.0 ? jump_threshold_.load() : request->jump_threshold,
    request->speed,
    &response->message
  );
}

void RobotControllerNode::stop_exec_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  move_group_->stop();

  response->success = true;
}
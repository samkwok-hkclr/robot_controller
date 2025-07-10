#include "robot_controller/robot_ctlr_node.hpp"

void RobotControllerNode::get_curr_joint_states(
  const std::shared_ptr<GetCurrentJointStates::Request> request, 
  std::shared_ptr<GetCurrentJointStates::Response> response)
{
  (void) request;

  std::optional<sensor_msgs::msg::JointState> joint_states = move_group_->get_curr_joint_states();

  if (!joint_states.has_value())
  {
    response->success = false;
    response->message = "Failed to get the joint states";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->joint_states = *joint_states;
  response->success = true;

  RCLCPP_INFO(get_logger(), "%s successully", __FUNCTION__);
}

void RobotControllerNode::get_curr_pose_cb(
  const std::shared_ptr<GetCurrentPose::Request> request, 
  std::shared_ptr<GetCurrentPose::Response> response)
{
  std::optional<geometry_msgs::msg::Pose> pose = move_group_->get_curr_pose(request->joint_name);

  if (!pose.has_value())
  {
    response->success = false;
    response->message = "Failed to get current pose for joint [" + request->joint_name + "]";
    RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    return;
  }

  response->pose = *pose;
  response->success = true;

  RCLCPP_INFO(get_logger(), "%s successully", __FUNCTION__);
}

void RobotControllerNode::push_pose_array_cb(
  const std::shared_ptr<PushPoseArray::Request> request, 
  std::shared_ptr<PushPoseArray::Response> response)
{
  try
  {
    pushed_waypoints_.insert(pushed_waypoints_.end(), request->poses.poses.begin(), request->poses.poses.end());
  }
  catch(const std::exception& e)
  {
    response->success = false;
    response->message = e.what();
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return;
  }
  
  response->success = true;

  RCLCPP_INFO(get_logger(), "%s successully", __FUNCTION__);
}

void RobotControllerNode::clear_pose_array_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  pushed_waypoints_.clear();

  response->success = true;
  
  RCLCPP_INFO(get_logger(), "%s successully", __FUNCTION__);
}

void RobotControllerNode::feedback_cb(
  rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr,
  const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "state: %s", feedback->state.c_str());
}
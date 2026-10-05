#include "robot_controller/robot_controller_node.hpp"

namespace robot_controller
{

void RobotControllerNode::cbRobotSpeed(
  const std::shared_ptr<RobotSpeed::Request> request,
  std::shared_ptr<RobotSpeed::Response> response)
{
  std_msgs::msg::Float32 msg;
  msg.data = static_cast<float>(request->speed);
  speed_pub_->publish(msg);
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbGetPose(
  const std::shared_ptr<GetPose::Request> request,
  std::shared_ptr<GetPose::Response> response)
{
  auto pose = move_group_->getPose(request->link_name);
  if (!pose) {
    response->success = false;
    response->message = "Failed to get current pose for link [" + request->link_name + "]";
    return;
  }
  response->pose = *pose;
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbGetJointStates(
  const std::shared_ptr<GetJointStates::Request>,
  std::shared_ptr<GetJointStates::Response> response)
{
  auto states = move_group_->getJointStates();
  if (!states) {
    response->success = false;
    response->message = "Failed to get joint states";
    return;
  }
  response->joint_states = std::move(*states);
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbGetJointLimits(
  const std::shared_ptr<GetJointLimits::Request>,
  std::shared_ptr<GetJointLimits::Response> response)
{
  auto limits = move_group_->getJointLimits();
  if (!limits) {
    response->success = false;
    response->message = "Failed to get joint limits";
    return;
  }
  response->joint_limits = std::move(*limits);
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbPushPoseArray(
  const std::shared_ptr<PushPoseArray::Request> request,
  std::shared_ptr<PushPoseArray::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(pushed_waypoints_mtx_);
    pushed_waypoints_.insert(
      pushed_waypoints_.end(),
      request->poses.poses.begin(), request->poses.poses.end());
  }
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbClearPoseArray(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  {
    std::lock_guard<std::mutex> lock(pushed_waypoints_mtx_);
    pushed_waypoints_.clear();
  }
  response->success = true;
  response->message = "ok";
}

}  // namespace robot_controller
#include "robot_controller/move_group_interface.hpp"
#include "robot_controller/bspline.hpp"

#include <algorithm>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_state/robot_state.h>

namespace robot_controller
{

using MoveGroupImpl = moveit::planning_interface::MoveGroupInterface;

// ---------------------------------------------------------------------------
MoveGroupInterface::MoveGroupInterface(rclcpp::Node & node, const std::string & name)
: node_(node)
{
  (void)name;  // retained for symmetry with the old API; unused
}

MoveGroupInterface::~MoveGroupInterface() = default;

// ---------------------------------------------------------------------------
bool MoveGroupInterface::init(
  const std::string & ns,
  const std::string & group_name,
  const std::string & eef_name,
  const std::string & ref_frame)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (group_name.empty()) {
    RCLCPP_ERROR(node_.get_logger(), "init: group_name is empty");
    return false;
  }

  try {
    if (ns.empty()) {
      move_group_ = std::make_unique<MoveGroupImpl>(node_.shared_from_this(), group_name);
    } else {
      std::string normalized_ns = ns;
      if (normalized_ns.front() != '/') {
        normalized_ns.insert(0, "/");
      }
      MoveGroupImpl::Options opt(group_name, MoveGroupImpl::ROBOT_DESCRIPTION, normalized_ns);
      move_group_ = std::make_unique<MoveGroupImpl>(node_.shared_from_this(), opt);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "init: %s", e.what());
    return false;
  }

  if (!eef_name.empty() && !move_group_->setEndEffectorLink(eef_name)) {
    RCLCPP_ERROR(node_.get_logger(), "init: setEndEffectorLink('%s') failed", eef_name.c_str());
    return false;
  }

  if (!ref_frame.empty()) {
    move_group_->setPoseReferenceFrame(ref_frame);
  }

  planning_scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  return true;
}

// ---------------------------------------------------------------------------
void MoveGroupInterface::setUseBspline(bool use, double step)
{
  std::lock_guard<std::mutex> lock(mtx_);
  use_bspline_ = use;
  bspline_step_ = (step > 0.0) ? step : 0.1;
}

// ---------------------------------------------------------------------------
void MoveGroupInterface::setDefaultSpeedScaling(double velocity, double acceleration)
{
  std::lock_guard<std::mutex> lock(mtx_);
  default_velocity_scaling_ = std::clamp(velocity, 0.0, 1.0);
  default_acceleration_scaling_ = std::clamp(acceleration, 0.0, 1.0);
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::validateScaling(double & value, const char * name) const
{
  if (value < 0.0 || value > 1.0) {
    RCLCPP_WARN(
      node_.get_logger(),
      "Invalid %s=%.4f; clamping to [0, 1]", name, value);
    value = std::clamp(value, 0.0, 1.0);
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
void MoveGroupInterface::resetStartState()
{
  if (move_group_) {
    move_group_->setStartStateToCurrentState();
  }
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::addCollisionObjects(
  std::vector<moveit_msgs::msg::CollisionObject> objects,
  const std::vector<moveit_msgs::msg::ObjectColor> & colors)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_ || !planning_scene_) {
    RCLCPP_ERROR(node_.get_logger(), "addCollisionObjects: not initialized");
    return false;
  }

  const std::string frame = move_group_->getPlanningFrame();
  for (auto & obj : objects) {
    obj.header.frame_id = frame;
  }

  try {
    planning_scene_->addCollisionObjects(objects, colors);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "addCollisionObjects: %s", e.what());
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::removeCollisionObjects(const std::vector<std::string> & object_ids)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_ || !planning_scene_) {
    return false;
  }

  // Synchronous removal — the caller expects the scene to be updated by the
  // time this call returns.
  std::vector<moveit_msgs::msg::CollisionObject> to_remove;
  to_remove.reserve(object_ids.size());
  for (const auto & id : object_ids) {
    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    to_remove.push_back(std::move(obj));
  }

  try {
    planning_scene_->applyCollisionObjects(to_remove);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "removeCollisionObjects: %s", e.what());
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::moveCollisionObject(
  const std::string & object_id,
  const geometry_msgs::msg::Pose & pose,
  bool is_mesh)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_ || !planning_scene_) {
    return false;
  }

  moveit_msgs::msg::CollisionObject obj;
  obj.id = object_id;
  obj.operation = moveit_msgs::msg::CollisionObject::MOVE;
  if (is_mesh) {
    obj.mesh_poses.push_back(pose);
  } else {
    obj.primitive_poses.push_back(pose);
  }

  try {
    planning_scene_->addCollisionObjects({obj}, {});
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "moveCollisionObject: %s", e.what());
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::applyAttachedCollisionObjects(
  const std::vector<moveit_msgs::msg::AttachedCollisionObject> & acos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!planning_scene_) {
    return false;
  }
  try {
    planning_scene_->applyAttachedCollisionObjects(acos);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "applyAttachedCollisionObjects: %s", e.what());
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
std::optional<std::map<std::string, moveit_msgs::msg::CollisionObject>>
MoveGroupInterface::getCollisionObjectsFromScene(const std::vector<std::string> & object_ids)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!planning_scene_) {
    RCLCPP_ERROR(node_.get_logger(), "getCollisionObjectsFromScene: not initialized");
    return std::nullopt;
  }
  try {
    return planning_scene_->getObjects(object_ids);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "getCollisionObjectsFromScene: %s", e.what());
    return std::nullopt;
  }
}

// ---------------------------------------------------------------------------
std::optional<geometry_msgs::msg::Pose>
MoveGroupInterface::getPose(const std::string & link_name) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_) {
    return std::nullopt;
  }
  try {
    return move_group_->getCurrentPose(link_name).pose;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_.get_logger(), "getPose: %s", e.what());
    return std::nullopt;
  }
}

// ---------------------------------------------------------------------------
std::optional<sensor_msgs::msg::JointState>
MoveGroupInterface::getJointStates() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_) {
    return std::nullopt;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = node_.now();
  msg.name = move_group_->getJointNames();
  msg.position = move_group_->getCurrentJointValues();
  return msg;
}

// ---------------------------------------------------------------------------
std::optional<std::vector<moveit_msgs::msg::JointLimits>>
MoveGroupInterface::getJointLimits() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_) {
    return std::nullopt;
  }

  auto state = move_group_->getCurrentState();
  if (!state) {
    return std::nullopt;
  }
  const auto * jmg = state->getJointModelGroup(move_group_->getName());
  if (!jmg) {
    return std::nullopt;
  }

  std::vector<moveit_msgs::msg::JointLimits> limits;
  for (const auto & joint_name : jmg->getJointModelNames()) {
    const auto * jm = jmg->getJointModel(joint_name);
    if (!jm) continue;
    auto bounds = jm->getVariableBoundsMsg();
    // Keep all variable bounds; MoveIt2 emits one entry per variable.
    for (auto & b : bounds) {
      if (b.joint_name.empty()) b.joint_name = joint_name;
      limits.push_back(std::move(b));
    }
  }

  if (limits.empty()) {
    return std::nullopt;
  }
  return limits;
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::planJointTarget(
  const std::vector<double> & joint_positions,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  moveit_msgs::msg::MoveItErrorCodes & error_code)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  move_group_->setJointValueTarget(joint_positions);

  MoveGroupImpl::Plan plan;
  const auto code = move_group_->plan(plan);
  resetStartState();

  error_code.val = code.val;
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }
  trajectory = plan.trajectory_;
  return true;
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::planPoseTarget(
  const geometry_msgs::msg::Pose & target_pose,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  moveit_msgs::msg::MoveItErrorCodes & error_code)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  move_group_->setPoseTarget(target_pose);

  MoveGroupImpl::Plan plan;
  const auto code = move_group_->plan(plan);
  resetStartState();

  error_code.val = code.val;
  if (code != moveit::core::MoveItErrorCode::SUCCESS) {
    return false;
  }
  trajectory = plan.trajectory_;
  return true;
}

// ---------------------------------------------------------------------------
double MoveGroupInterface::planCartesianPath(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  double eef_step,
  double jump_threshold,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  bool avoid_collisions,
  moveit_msgs::msg::MoveItErrorCodes * error_code)
{
  if (waypoints.empty()) {
    return 0.0;
  }

  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_) {
    return 0.0;
  }

  auto compute = [&](const std::vector<geometry_msgs::msg::Pose> & wps) {
    return move_group_->computeCartesianPath(
      wps, eef_step, jump_threshold, trajectory, avoid_collisions, error_code);
  };

  if (!use_bspline_) {
    return compute(waypoints);
  }

  // Fallback chain: bspline without enrichment -> bspline with enrichment ->
  // original waypoints.
  try {
    Bspline spline_no_enrich(waypoints, false);
    auto smoothed = spline_no_enrich.interpolate(bspline_step_);
    double fraction = compute(smoothed);
    if (fraction >= 1.0) {
      return fraction;
    }

    RCLCPP_WARN(
      node_.get_logger(),
      "Bspline (no enrichment) fraction=%.3f; retrying with enrichment", fraction);

    Bspline spline_enrich(waypoints, true);
    auto smoothed_enriched = spline_enrich.interpolate(bspline_step_);
    fraction = compute(smoothed_enriched);
    if (fraction >= 1.0) {
      return fraction;
    }

    RCLCPP_WARN(
      node_.get_logger(),
      "Bspline (with enrichment) fraction=%.3f; falling back to linear path", fraction);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_.get_logger(), "Bspline failed (%s); falling back to linear path", e.what());
  }

  return compute(waypoints);
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::planJointWaypointsFromPoses(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  moveit_msgs::msg::MoveItErrorCodes & error_code)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_ || waypoints.empty()) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  auto state = move_group_->getCurrentState();
  const auto * jmg = state->getJointModelGroup(move_group_->getName());
  if (!jmg) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  moveit_msgs::msg::RobotTrajectory whole;
  bool first = true;

  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const auto & pose = waypoints[i];
    move_group_->setPoseTarget(pose);

    MoveGroupImpl::Plan partial;
    const auto code = move_group_->plan(partial);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "planJointWaypointsFromPoses: segment %zu failed (val=%d)", i, code.val);
      resetStartState();
      error_code.val = code.val;
      return false;
    }

    whole.joint_trajectory.joint_names = partial.trajectory_.joint_trajectory.joint_names;
    const auto & pts = partial.trajectory_.joint_trajectory.points;
    if (first) {
      whole.joint_trajectory.points.push_back(pts.front());
      first = false;
    }
    whole.joint_trajectory.points.insert(
      whole.joint_trajectory.points.end(), pts.begin() + 1, pts.end());

    // Prepare IK-based start state for the next segment.
    if (i + 1 < waypoints.size()) {
      moveit::core::RobotState start_state(*move_group_->getCurrentState());
      if (!start_state.setFromIK(jmg, pose, move_group_->getEndEffectorLink())) {
        RCLCPP_ERROR(node_.get_logger(), "planJointWaypointsFromPoses: IK failed at segment %zu", i);
        resetStartState();
        error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
        return false;
      }
      move_group_->setStartState(start_state);
    }
  }

  resetStartState();

  if (!applyTimeParameterization(
        whole, default_velocity_scaling_, default_acceleration_scaling_))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  trajectory = std::move(whole);
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::planJointWaypoints(
  const std::vector<std::vector<double>> & joint_waypoints,
  moveit_msgs::msg::RobotTrajectory & trajectory,
  moveit_msgs::msg::MoveItErrorCodes & error_code)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_ || joint_waypoints.empty()) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  auto state = move_group_->getCurrentState();
  const auto * jmg = state->getJointModelGroup(move_group_->getName());
  if (!jmg) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  moveit_msgs::msg::RobotTrajectory whole;
  bool first = true;

  for (std::size_t i = 0; i < joint_waypoints.size(); ++i) {
    moveit::core::RobotState start_state(*move_group_->getCurrentState());
    std::vector<double> start_joints;
    if (i > 0) {
      start_joints = joint_waypoints[i - 1];
      start_state.setJointGroupPositions(jmg, start_joints);
    }
    move_group_->setStartState(start_state);
    move_group_->setJointValueTarget(joint_waypoints[i]);

    MoveGroupImpl::Plan partial;
    const auto code = move_group_->plan(partial);
    if (code != moveit::core::MoveItErrorCode::SUCCESS) {
      resetStartState();
      error_code.val = code.val;
      return false;
    }

    whole.joint_trajectory.joint_names = partial.trajectory_.joint_trajectory.joint_names;
    const auto & pts = partial.trajectory_.joint_trajectory.points;
    if (first) {
      whole.joint_trajectory.points.push_back(pts.front());
      first = false;
    }
    whole.joint_trajectory.points.insert(
      whole.joint_trajectory.points.end(), pts.begin() + 1, pts.end());
  }

  resetStartState();

  if (!applyTimeParameterization(
        whole, default_velocity_scaling_, default_acceleration_scaling_))
  {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return false;
  }

  trajectory = std::move(whole);
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

// ---------------------------------------------------------------------------
moveit_msgs::msg::MoveItErrorCodes MoveGroupInterface::execute(
  const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  std::lock_guard<std::mutex> lock(mtx_);
  moveit_msgs::msg::MoveItErrorCodes result;
  if (!move_group_) {
    result.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    return result;
  }
  const auto code = move_group_->execute(trajectory);
  result.val = code.val;
  return result;
}

// ---------------------------------------------------------------------------
void MoveGroupInterface::stop()
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (move_group_) {
    move_group_->stop();
  }
}

// ---------------------------------------------------------------------------
bool MoveGroupInterface::applyTimeParameterization(
  moveit_msgs::msg::RobotTrajectory & trajectory,
  double velocity_scaling,
  double acceleration_scaling)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!move_group_) {
    return false;
  }

  validateScaling(velocity_scaling, "velocity_scaling");
  validateScaling(acceleration_scaling, "acceleration_scaling");

  moveit::core::RobotState current(*move_group_->getCurrentState());
  robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
  rt.setRobotTrajectoryMsg(current, trajectory);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling)) {
    RCLCPP_ERROR(node_.get_logger(), "TOTG failed");
    return false;
  }
  rt.getRobotTrajectoryMsg(trajectory);
  return true;
}

// ---------------------------------------------------------------------------
std::string MoveGroupInterface::vectorToStr(const std::vector<double> & v)
{
  std::ostringstream ss;
  ss << '[';
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) ss << ", ";
    ss << v[i];
  }
  ss << ']';
  return ss.str();
}

}  // namespace robot_controller
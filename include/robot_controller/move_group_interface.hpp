#ifndef ROBOT_CONTROLLER__MOVE_GROUP_INTERFACE_HPP_
#define ROBOT_CONTROLLER__MOVE_GROUP_INTERFACE_HPP_

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/joint_limits.hpp>
#include <moveit_msgs/msg/object_color.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

namespace robot_controller
{

/**
 * Non-ROS class wrapping MoveIt2's MoveGroupInterface and
 * PlanningSceneInterface. All public methods are thread-safe (a single
 * mutex guards the MoveIt2 objects).
 *
 * The owning rclcpp::Node is only used for logging and parameter access.
 */
class MoveGroupInterface
{
public:
  MoveGroupInterface(rclcpp::Node & node, const std::string & name);
  ~MoveGroupInterface();

  MoveGroupInterface(const MoveGroupInterface &) = delete;
  MoveGroupInterface & operator=(const MoveGroupInterface &) = delete;

  // ---- Initialization -----------------------------------------------------
  bool init(
    const std::string & ns,
    const std::string & group_name,
    const std::string & eef_name,
    const std::string & ref_frame);

  void setUseBspline(bool use, double step);
  void setDefaultSpeedScaling(double velocity, double acceleration);

  // ---- Collision-object operations ---------------------------------------
  bool addCollisionObjects(
    std::vector<moveit_msgs::msg::CollisionObject> objects,
    const std::vector<moveit_msgs::msg::ObjectColor> & colors = {});

  bool removeCollisionObjects(const std::vector<std::string> & object_ids);

  bool moveCollisionObject(
    const std::string & object_id,
    const geometry_msgs::msg::Pose & pose,
    bool is_mesh);

  bool applyAttachedCollisionObjects(
    const std::vector<moveit_msgs::msg::AttachedCollisionObject> & acos);

  std::optional<std::map<std::string, moveit_msgs::msg::CollisionObject>>
  getCollisionObjectsFromScene(const std::vector<std::string> & object_ids = {});

  // ---- State queries ------------------------------------------------------
  std::optional<geometry_msgs::msg::Pose> getPose(const std::string & link_name = "") const;
  std::optional<sensor_msgs::msg::JointState> getJointStates() const;
  std::optional<std::vector<moveit_msgs::msg::JointLimits>> getJointLimits() const;

  // ---- Planning -----------------------------------------------------------
  /// Plan a joint-space motion to a single joint target.
  bool planJointTarget(
    const std::vector<double> & joint_positions,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    moveit_msgs::msg::MoveItErrorCodes & error_code);

  /// Plan a Cartesian motion to a single pose.
  bool planPoseTarget(
    const geometry_msgs::msg::Pose & target_pose,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    moveit_msgs::msg::MoveItErrorCodes & error_code);

  /// Cartesian path through waypoints. Fills fraction and (optional) error code.
  double planCartesianPath(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    double eef_step,
    double jump_threshold,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    bool avoid_collisions,
    moveit_msgs::msg::MoveItErrorCodes * error_code = nullptr);

  /// Joint-space plan through Cartesian waypoints (IK + plan per segment).
  bool planJointWaypointsFromPoses(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    moveit_msgs::msg::MoveItErrorCodes & error_code);

  /// Joint-space plan through joint waypoints.
  bool planJointWaypoints(
    const std::vector<std::vector<double>> & joint_waypoints,
    moveit_msgs::msg::RobotTrajectory & trajectory,
    moveit_msgs::msg::MoveItErrorCodes & error_code);

  // ---- Execution ----------------------------------------------------------
  moveit_msgs::msg::MoveItErrorCodes execute(
    const moveit_msgs::msg::RobotTrajectory & trajectory);

  void stop();

  // ---- Trajectory post-processing ----------------------------------------
  bool applyTimeParameterization(
    moveit_msgs::msg::RobotTrajectory & trajectory,
    double velocity_scaling,
    double acceleration_scaling);

  // ---- Helpers ------------------------------------------------------------
  static std::string vectorToStr(const std::vector<double> & v);

private:
  bool validateScaling(double & value, const char * name) const;
  void resetStartState();

  rclcpp::Node & node_;
  mutable std::mutex mtx_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_;

  bool use_bspline_{false};
  double bspline_step_{0.1};
  double default_velocity_scaling_{1.0};
  double default_acceleration_scaling_{1.0};
};

}  // namespace robot_controller

#endif  // ROBOT_CONTROLLER__MOVE_GROUP_INTERFACE_HPP_
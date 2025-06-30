#ifndef MOVE_GROUP_CONTROLLER_HPP__
#define MOVE_GROUP_CONTROLLER_HPP__

#pragma once

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

#include "geometry_msgs/msg/pose_array.hpp"

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
using PlanningSceneInterface = moveit::planning_interface::PlanningSceneInterface;

class MoveGroup
{
public:
  MoveGroup(std::shared_ptr<rclcpp::Node> node);
  ~MoveGroup();
  
  bool config_robot(std::string robot_description_path, std::string robot_semantic_path, std::string kinematics_path);
  bool init_move_group(std::string group_name, std::string end_effector_name);
  void set_use_bspline(bool use, double step);

  bool add_collision_objects(
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects,
    const std::vector<moveit_msgs::msg::ObjectColor>& object_colors = std::vector<moveit_msgs::msg::ObjectColor>());
  bool remove_collision_objects(const std::vector<std::string>& object_ids);
  bool move_collision_object(std::string key, const geometry_msgs::msg::Pose& pose, bool is_mesh);

  bool apply_attached_collision_objects(const std::vector<moveit_msgs::msg::AttachedCollisionObject>& attached_collision_objects);
  std::map<std::string, moveit_msgs::msg::CollisionObject> get_collision_objects_from_scene(
    const std::vector<std::string>& object_ids = std::vector<std::string>());

  geometry_msgs::msg::Pose get_curr_pose(std::string joint_name = "");
  sensor_msgs::msg::JointState get_curr_joint_states();
  
  moveit::core::MoveItErrorCode execute(
    const moveit_msgs::msg::RobotTrajectory& trajectory);
  void stop();

  double compute_cartesian_path(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const double eef_step, 
    const double jump_threshold,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    bool avoid_collisions = true,
    moveit_msgs::msg::MoveItErrorCodes* error_code = nullptr);

  bool compute_joint_path(const std::vector<geometry_msgs::msg::Pose>& waypoints, moveit_msgs::msg::RobotTrajectory& trajectory);
  bool compute_joint_path(const std::vector<std::vector<double>>& joint_waypoints, moveit_msgs::msg::RobotTrajectory& trajectory);

  moveit::core::MoveItErrorCode execute_pose(const geometry_msgs::msg::Pose& pose);
  moveit::core::MoveItErrorCode execute_joints(const std::vector<double>& joints);

  void set_max_velocity_scaling_factor(double max_velocity_scaling_factor);
  void set_max_acceleration_scaling_factor(double max_acceleration_scaling_factor);

  bool compute_timestamps_totg(
    moveit_msgs::msg::RobotTrajectory& trajectory, 
    const double max_velocity_scaling_factor = 1.0, 
    const double max_acceleration_scaling_factor = 1.0);

  std::string vec_to_str(const std::vector<double>& vec);

private:
  std::string group_name_;
  // void asyncSpinner();
  // void configRobotFromFile(std::string name, std::string path);
  // void configKinematics(std::string path);
  // std::string loadFile(std::string path);

  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<MoveGroupInterface> move_group_;
  std::unique_ptr<PlanningSceneInterface> planning_scene_;

  std::mutex execution_mtx_;
  bool use_bspline_ = false;
  double bspline_step_ = 0.1;
};

#endif
#ifndef ROBOT_CONTROLLER_NODE__
#define ROBOT_CONTROLLER_NODE__

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <Eigen/Geometry>

#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/srv/get_cartesian_path.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "sensor_msgs/msg/region_of_interest.hpp"

#include "robot_controller_msgs/msg/collision_object_map.hpp"

#include "robot_controller_msgs/srv/add_collision_objects.hpp"
#include "robot_controller_msgs/srv/apply_attached_collision_objects.hpp"
#include "robot_controller_msgs/srv/execute_joints.hpp"
#include "robot_controller_msgs/srv/execute_joint_waypoints.hpp"
#include "robot_controller_msgs/srv/execute_pose.hpp"
#include "robot_controller_msgs/srv/execute_waypoints.hpp"
#include "robot_controller_msgs/srv/get_collision_objects_from_scene.hpp"
#include "robot_controller_msgs/srv/get_current_joint_states.hpp"
#include "robot_controller_msgs/srv/get_current_pose.hpp"
#include "robot_controller_msgs/srv/move_collision_objects.hpp"
#include "robot_controller_msgs/srv/push_pose_array.hpp"
#include "robot_controller_msgs/srv/remove_collision_objects.hpp"

#include "move_group.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class RobotControllerNode : public rclcpp::Node
{
  using Trigger = std_srvs::srv::Trigger;

  using RegionOfInterest = sensor_msgs::msg::RegionOfInterest;

  using GetCartesianPath = moveit_msgs::srv::GetCartesianPath;
  using ExecuteTrajectory = moveit_msgs::action::ExecuteTrajectory;

  using CollisionObjectMap = robot_controller_msgs::msg::CollisionObjectMap;

  using AddCollisionObjects = robot_controller_msgs::srv::AddCollisionObjects;
  using ApplyAttachedCollisionObjects = robot_controller_msgs::srv::ApplyAttachedCollisionObjects;
  using ExecuteJoints = robot_controller_msgs::srv::ExecuteJoints;
  using ExecuteJointWaypoints = robot_controller_msgs::srv::ExecuteJointWaypoints;
  using ExecutePose = robot_controller_msgs::srv::ExecutePose;
  using ExecuteWaypoints = robot_controller_msgs::srv::ExecuteWaypoints;
  using GetCollisionObjectsFromScene = robot_controller_msgs::srv::GetCollisionObjectsFromScene;
  using GetCurrentJointStates = robot_controller_msgs::srv::GetCurrentJointStates;
  using GetCurrentPose = robot_controller_msgs::srv::GetCurrentPose;
  using MoveCollisionObjects = robot_controller_msgs::srv::MoveCollisionObjects;
  using PushPoseArray = robot_controller_msgs::srv::PushPoseArray;
  using RemoveCollisionObjects = robot_controller_msgs::srv::RemoveCollisionObjects;

public:
  explicit RobotControllerNode(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor);
  ~RobotControllerNode() = default;
  
  // int config(bool simulation, const RobotConfig& config);
  // void configSensingRegion(const RegionOfInterest& sensing_region, RobotLeftSensingRegionCallback cb);

  // // Arm control and get info
  // int executePick(int robot_id, const geometry_msgs::msg::PoseArray& pick_waypoints, float speed);
  // int executePlace(int robot_id, const geometry_msgs::msg::PoseArray& place_waypoints, float speed);
  
  // // set simulation scale
  // int setSimulationScale(int scale);
  
  bool exec_waypoints(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const double speed,
    std::string *ret_msg = nullptr);
  bool exec_waypoints(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    const double eef_step,
    const double jump_threshold,
    const double speed,
    std::string *ret_msg = nullptr);
  bool exec_pushed_waypoints(std::string *ret_msg = nullptr);

  // ============== Debug Services ==============
  void testing_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);

  // ============== Execution Services ==============
  void exec_joints_cb(
    const std::shared_ptr<ExecuteJoints::Request> request, 
    std::shared_ptr<ExecuteJoints::Response> response);
  void exec_pose_cb(
    const std::shared_ptr<ExecutePose::Request> request, 
    std::shared_ptr<ExecutePose::Response> response);
  void exec_waypoints_cb(
    const std::shared_ptr<ExecuteWaypoints::Request> request, 
    std::shared_ptr<ExecuteWaypoints::Response> response);
  void exec_joint_waypoints_cb(
    const std::shared_ptr<ExecuteJointWaypoints::Request> request, 
    std::shared_ptr<ExecuteJointWaypoints::Response> response);
  void stop_exec_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);

  // ============== Collision Objects Services ==============
  void add_collision_obj_cb(
    const std::shared_ptr<AddCollisionObjects::Request> request, 
    std::shared_ptr<AddCollisionObjects::Response> response);
  void remove_collision_obj_cb(
    const std::shared_ptr<RemoveCollisionObjects::Request> request, 
    std::shared_ptr<RemoveCollisionObjects::Response> response);
  void apply_attached_collision_obj_cb(
    const std::shared_ptr<ApplyAttachedCollisionObjects::Request> request, 
    std::shared_ptr<ApplyAttachedCollisionObjects::Response> response);
  void move_collision_obj_cb(
    const std::shared_ptr<MoveCollisionObjects::Request> request, 
    std::shared_ptr<MoveCollisionObjects::Response> response);
  void get_collision_obj_from_scene_cb(
    const std::shared_ptr<GetCollisionObjectsFromScene::Request> request, 
    std::shared_ptr<GetCollisionObjectsFromScene::Response> response);

  // ============== Utilities Services ==============
  void get_curr_pose_cb(
    const std::shared_ptr<GetCurrentPose::Request> request, 
    std::shared_ptr<GetCurrentPose::Response> response);
  void get_curr_joint_states(
    const std::shared_ptr<GetCurrentJointStates::Request> request, 
    std::shared_ptr<GetCurrentJointStates::Response> response);

  void push_pose_array_cb(
    const std::shared_ptr<PushPoseArray::Request> request, 
    std::shared_ptr<PushPoseArray::Response> response);
  void clear_pose_array_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);

  void feedback_cb(
    rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr,
    const std::shared_ptr<const ExecuteTrajectory::Feedback> feedback);
  
private:
  // bool stop_flag_ = false;
  
  // RobotConfig robot_config_;
  std::atomic<double> eef_step_;
  std::atomic<double> jump_threshold_;

  std::vector<geometry_msgs::msg::Pose> pushed_waypoints_{};
  std::unique_ptr<MoveGroup> move_group_;
  // RobotLeftSensingRegionCallback left_region_cb_;

  // //For monitoring robot states
  // std::unique_ptr<std::thread> monitor_thread_;

  // //Simulation scale control
  // std::unique_ptr<SimRobotControl> sim_control_;

  //Remove dupilicated pose in the waypoints
  void process_waypoints(std::vector<geometry_msgs::msg::Pose>& waypoints);
  bool are_poses_equal(
    const geometry_msgs::msg::Pose& pose1,
    const geometry_msgs::msg::Pose& pose2,
    double pos_thd = 1e-6,
    double ori_thd = 1e-6);

  // void monitorSensingRegion(const RegionOfInterest& sensing_region);

  // CLRError executePushedWayPoints(int robot_id);
  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr exec_srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr col_obj_srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr get_col_obj_srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_cli_cbg_;

  // ============== Publishers ==============

  // ============== Service Clients ==============
  rclcpp::Client<GetCartesianPath>::SharedPtr get_cartesian_path_cli_;

  // ============== Debug Services ==============
  rclcpp::Service<Trigger>::SharedPtr testing_srv_;

  // ============== Execution Services ==============
  rclcpp::Service<ExecuteJoints>::SharedPtr exec_joints_srv_;
  rclcpp::Service<ExecutePose>::SharedPtr exec_pose_srv_;
  rclcpp::Service<ExecuteWaypoints>::SharedPtr exec_waypoints_srv_;
  rclcpp::Service<ExecuteJointWaypoints>::SharedPtr exec_joint_waypoints_srv_;
  rclcpp::Service<Trigger>::SharedPtr stop_exec_srv_;
  
  // ============== Collision Objects Services ==============
  rclcpp::Service<AddCollisionObjects>::SharedPtr add_collision_obj_srv_;
  rclcpp::Service<RemoveCollisionObjects>::SharedPtr remove_collision_obj_srv_;
  rclcpp::Service<ApplyAttachedCollisionObjects>::SharedPtr apply_attached_collision_obj_srv_;
  rclcpp::Service<MoveCollisionObjects>::SharedPtr move_collision_obj_srv_;
  rclcpp::Service<GetCollisionObjectsFromScene>::SharedPtr get_collision_obj_from_secne_srv_;

  // ============== General Services ==============
  rclcpp::Service<GetCurrentPose>::SharedPtr get_curr_pose_srv_;
  rclcpp::Service<GetCurrentJointStates>::SharedPtr get_curr_joint_states_srv_;
  rclcpp::Service<PushPoseArray>::SharedPtr push_pose_arr_srv_;
  rclcpp::Service<Trigger>::SharedPtr clear_pose_arr_srv_;

  rclcpp_action::Client<ExecuteTrajectory>::SharedPtr exe_trajectory_cli_;

};

#endif // ROBOT_CONTROLLER_NODE__
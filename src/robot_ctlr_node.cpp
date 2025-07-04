#include "robot_controller/robot_ctlr_node.hpp"

RobotControllerNode::RobotControllerNode(
  const rclcpp::NodeOptions& options,
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
  : Node("robot_controller_node", options)
{
  std::string group_name; 
  std::string eef_name;
  std::string description_file;
  std::string semantic_file;
  std::string kinematics_file;

  declare_parameter<std::string>("group_name", "");
  declare_parameter<std::string>("eef_name", "");
  declare_parameter<std::string>("description_file", "");
  declare_parameter<std::string>("semantic_file", "");
  declare_parameter<std::string>("kinematics_file", "");
  declare_parameter<double>("default_eef_step", 0.0);
  declare_parameter<double>("default_jump_threshold", 0.0);

  get_parameter("group_name", group_name);
  get_parameter("eef_name", eef_name);
  get_parameter("description_file", description_file); // FIXME: It is empty
  get_parameter("semantic_file", semantic_file); // FIXME: It is empty
  get_parameter("kinematics_file", kinematics_file); // FIXME: It is empty
  eef_step_ = get_parameter("default_eef_step").as_double();
  jump_threshold_ = get_parameter("default_jump_threshold").as_double();

  auto node = rclcpp::Node::make_shared("move_group_node");
  move_group_ = std::make_unique<MoveGroup>(node);
  executor->add_node(node->get_node_base_interface());

  if (!move_group_->config_robot(description_file, semantic_file, kinematics_file))
  {    
    RCLCPP_INFO(get_logger(), "move group config Failed.");
    rclcpp::shutdown();
    return;
  }

  if (!move_group_->init_move_group(group_name, eef_name))
  {
    RCLCPP_INFO(get_logger(), "move group init Failed.");
    rclcpp::shutdown();
    return;
  }

  srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // ============== Debug Services ==============
  testing_srv_ = create_service<Trigger>(
    "testing", 
    std::bind(&RobotControllerNode::testing_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  // ============== Execution Services ==============
  exec_joints_srv_ = create_service<ExecuteJoints>(
    "execute_joints", 
    std::bind(&RobotControllerNode::exec_joints_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  exec_pose_srv_ = create_service<ExecutePose>(
    "execute_pose", 
    std::bind(&RobotControllerNode::exec_pose_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  exec_waypoints_srv_ = create_service<ExecuteWaypoints>(
    "execute_waypoints", 
    std::bind(&RobotControllerNode::exec_waypoints_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  stop_exec_srv_ = create_service<Trigger>(
    "stop_execution", 
    std::bind(&RobotControllerNode::stop_exec_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  // ============== Collision Objects Services ==============
  add_collision_obj_srv_ = create_service<AddCollisionObjects>(
    "add_collision_objects", 
    std::bind(&RobotControllerNode::add_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  remove_collision_obj_srv_ = create_service<RemoveCollisionObjects>(
    "remove_collision_objects", 
    std::bind(&RobotControllerNode::remove_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  apply_attached_collision_obj_srv_ = create_service<ApplyAttachedCollisionObjects>(
    "apply_attached_collision_objects", 
    std::bind(&RobotControllerNode::apply_attached_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  move_collision_obj_srv_ = create_service<MoveCollisionObjects>(
    "move_collision_objects", 
    std::bind(&RobotControllerNode::move_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  get_collision_obj_from_secne_srv_ = create_service<GetCollisionObjectsFromScene>(
    "get_collision_objects_from_sence", 
    std::bind(&RobotControllerNode::get_collision_obj_from_scene_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);
    
  // ============== Utilities Services ==============
  get_curr_joint_states_srv_ = create_service<GetCurrentJointStates>(
    "get_current_joint_states", 
    std::bind(&RobotControllerNode::get_curr_joint_states, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  get_curr_pose_srv_ = create_service<GetCurrentPose>(
    "get_current_pose", 
    std::bind(&RobotControllerNode::get_curr_pose_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  push_pose_arr_srv_ = create_service<PushPoseArray>(
    "push_pose_array", 
    std::bind(&RobotControllerNode::push_pose_array_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  clear_pose_arr_srv_ = create_service<Trigger>(
    "clear_pose_array", 
    std::bind(&RobotControllerNode::clear_pose_array_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  exe_trajectory_cli_ = rclcpp_action::create_client<ExecuteTrajectory>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "execute_trajectory",
    action_cli_cbg_);

  RCLCPP_INFO(get_logger(), "Robot Controller Node is up.");
}

bool RobotControllerNode::are_poses_equal(
  const geometry_msgs::msg::Pose &pose1, 
  const geometry_msgs::msg::Pose &pose2, 
  double pos_tol, 
  double ang_tol)
{
  Eigen::Vector3d pos1(pose1.position.x, pose1.position.y, pose1.position.z);
  Eigen::Vector3d pos2(pose2.position.x, pose2.position.y, pose2.position.z);

  if ((pos1 - pos2).norm() > pos_tol) 
    return false;
  
  // Quaternion check (normalized)
  Eigen::Quaterniond q1(
    pose1.orientation.w,
    pose1.orientation.x,
    pose1.orientation.y,
    pose1.orientation.z
  );
  Eigen::Quaterniond q2(
    pose2.orientation.w,
    pose2.orientation.x,
    pose2.orientation.y,
    pose2.orientation.z
  );

  // Check if q1 and q2 represent the same rotation (allowing for sign flips)
  return (q1.angularDistance(q2) < ang_tol);
}

void RobotControllerNode::process_waypoints(std::vector<geometry_msgs::msg::Pose>& waypoints)
{
  if (waypoints.empty()) 
    return;
    
  std::vector<geometry_msgs::msg::Pose> filtered_waypoints;
  filtered_waypoints.reserve(waypoints.size());  // Reserve maximum possible size
    
  // Always keep the first waypoint
  filtered_waypoints.push_back(waypoints.front());
    
  // Process remaining waypoints
  for (auto it = waypoints.begin() + 1; it != waypoints.end(); ++it) 
  {
    if (!are_poses_equal(*it, filtered_waypoints.back())) 
    {
      filtered_waypoints.push_back(*it);
    }
  }

  waypoints.swap(filtered_waypoints);
}

bool RobotControllerNode::exec_waypoints(
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  const double speed,
  std::string *ret_msg)
{
  return exec_waypoints(waypoints, eef_step_.load(), jump_threshold_.load(), speed, ret_msg);
}

bool RobotControllerNode::exec_waypoints(
  const std::vector<geometry_msgs::msg::Pose>& input_waypoints,
  const double eef_step,
  const double jump_threshold,
  const double speed,
  std::string *ret_msg)
{
  std::vector<geometry_msgs::msg::Pose> waypoints;

  if (!pushed_waypoints_.empty())
  {
    RCLCPP_INFO(get_logger(), "Has pushed_waypoints!!!");
    waypoints.insert(waypoints.end(), pushed_waypoints_.begin(), pushed_waypoints_.end());
  }

  waypoints.insert(waypoints.end(), input_waypoints.begin(), input_waypoints.end());

  process_waypoints(waypoints);

  if (waypoints.size() == 0)
  {
    const std::string err_msg =  "Input waypoints is empty";
    *ret_msg = err_msg;

    RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "eef_stp: %.4f jump_threshold: %.4f", eef_step, jump_threshold);

  for (auto& pose : waypoints)
  {
    auto& q = pose.orientation;
    // Calculate squared norm first (more numerically stable)
    const double norm_squared = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    
    // Only normalize if norm is non-zero and not already normalized
    if (norm_squared > 0.0 && std::abs(norm_squared - 1.0) > 1e-6) 
    {
      const double norm = std::sqrt(norm_squared);
      q.x /= norm;
      q.y /= norm;
      q.z /= norm;
      q.w /= norm;

      RCLCPP_INFO(get_logger(), "normorlized a pose!");
    }
  }

  for (const auto& pose : waypoints)
  {
    RCLCPP_INFO(get_logger(), "--- waypoints: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f] ", 
      pose.position.x, pose.position.y, pose.position.z, 
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  }

  moveit_msgs::msg::RobotTrajectory trajectory;
  moveit_msgs::msg::MoveItErrorCodes errCodes;

  double fraction = move_group_->compute_cartesian_path(
    waypoints, 
    eef_step,
    jump_threshold, 
    trajectory, 
    true, 
    &errCodes);
    
  RCLCPP_INFO(get_logger(), "Compute Cartesian Path fraction: %.6f", fraction);

  if (fraction < 1.0)
  {
    for (auto js : trajectory.joint_trajectory.points.back().positions)
    {
      RCLCPP_ERROR(get_logger(), "last point >> %.2f", js * 180.0 / 3.14159);
    }
    *ret_msg = "fraction < 1.0";
    return false;
  }

  trajectory.joint_trajectory.header.frame_id = std::to_string(speed);

  RCLCPP_WARN(get_logger(), "Scaling trajectory with robot speed: %.4f", speed / 100.0);
  move_group_->compute_timestamps_totg(trajectory, speed / 100.0, speed / 100.0);

  RCLCPP_ERROR(get_logger(), "******** duration: %d.%d", 
    trajectory.joint_trajectory.points.back().time_from_start.sec, 
    trajectory.joint_trajectory.points.back().time_from_start.nanosec);

  moveit::core::MoveItErrorCode code = move_group_->execute(trajectory);

  if (code != 0 && code != moveit::core::MoveItErrorCode::SUCCESS)
  {
    const std::string err_msg = "Execute failed with error: " + std::to_string(code.val);
    *ret_msg = err_msg;

    RCLCPP_WARN(get_logger(), "%s", err_msg.c_str());
    return false;
  } 

  return true; 
}

bool RobotControllerNode::exec_pushed_waypoints(std::string *ret_msg)
{
  if (pushed_waypoints_.empty())
  {
    *ret_msg = "No waypoints available for execution";
    return false;
  }

  auto waypoints = std::exchange(pushed_waypoints_, {});

  // Execute with default velocity scaling (100%)
  const bool success = exec_waypoints(waypoints, 100.0);

  if (!success && ret_msg) 
  {
    *ret_msg = "Failed to execute waypoints trajectory";
  }

  return success;
}

void RobotControllerNode::testing_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  
  // auto goal_msg = ExecuteTrajectory::Goal();
  // goal_msg.trajectory = trajectory;

  // auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
  // send_goal_options.feedback_callback = std::bind(&RobotControllerNode::feedback_cb, this, _1, _2);
  // auto goal_handle_future = exe_trajectory_cli_->async_send_goal(goal_msg, send_goal_options);

  // goal_handle_future.wait();
  
  // rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr goal_handle = goal_handle_future.get();

  // auto result_future = exe_trajectory_cli_->async_get_result(goal_handle);
  // result_future.wait();
  
  response->success = true;
}


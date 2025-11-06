#include "robot_controller/robot_ctlr.hpp"

RobotController::RobotController(
  const rclcpp::NodeOptions& options,
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor)
: Node("robot_controller", options)
{
  declare_parameter<std::string>("move_group_namespace", "");
  declare_parameter<std::string>("group_name", "");
  declare_parameter<std::string>("eef_name", "");
  declare_parameter<std::string>("ref_frame", "");

  declare_parameter<double>("default_eef_step", 0.01);
  declare_parameter<double>("default_jump_threshold", 5.0);

  get_parameter("move_group_namespace", move_group_ns_);
  get_parameter("group_name", group_name_);
  get_parameter("eef_name", eef_name_);
  get_parameter("ref_frame", ref_frame_);

  eef_step_ = get_parameter("default_eef_step").as_double();
  jump_threshold_ = get_parameter("default_jump_threshold").as_double();

  rclcpp::NodeOptions move_group_options = options;
  // move_group_options.use_global_arguments(false);

  if (move_group_ns_.empty())
  {
    RCLCPP_INFO(get_logger(), "move_group_namespace is empty.");
  }
  else
  {
    // move_group_options.append_parameter_override("robot_description",  "/" + move_group_ns_ + "/robot_description");
    // move_group_options.append_parameter_override("robot_description_semantic", "/" + move_group_ns_ + "/robot_description_semantic");
    // move_group_options.append_parameter_override("robot_description_kinematics", "/" + move_group_ns_ + "/robot_description_kinematics");
  }
  move_group_options.automatically_declare_parameters_from_overrides(true);
  
  move_group_ = std::make_shared<MoveGroup>(move_group_options, group_name_);
  executor->add_node(move_group_->get_node_base_interface());

  if (!move_group_->init_move_group(move_group_ns_, group_name_, eef_name_, ref_frame_))
  {
    RCLCPP_INFO(get_logger(), "move group init Failed.");
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(get_logger(), "Robot Controller Node - initiated move group");

  srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  exec_srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  col_obj_srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  get_col_obj_srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  RCLCPP_INFO(get_logger(), "Robot Controller Node - initiated callback groups");
  
  // ============== Timers ==============
  pose_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&RobotController::pose_pub_cb, this),
    timer_cbg_);

  pose_pub_timer_->cancel(); // not used

  RCLCPP_INFO(get_logger(), "Robot Controller Node - initiated timers");
  
  // ============== Publishers ==============
  speed_pub_ = create_publisher<Float32>("robot_speed", 10);
  curr_pose_pub_ = create_publisher<Pose>("pose", 10);

  RCLCPP_INFO(get_logger(), "Robot Controller Node - initiated publishers");

  // ============== Debug Services ==============
  testing_srv_ = create_service<Trigger>(
    "testing", 
    std::bind(&RobotController::testing_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  // ============== Execution Services ==============
  exec_joints_srv_ = create_service<ExecuteJoints>(
    "execute_joints", 
    std::bind(&RobotController::exec_joints_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    exec_srv_ser_cbg_);

  exec_pose_srv_ = create_service<ExecutePose>(
    "execute_pose", 
    std::bind(&RobotController::exec_pose_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    exec_srv_ser_cbg_);

  exec_waypoints_srv_ = create_service<ExecuteWaypoints>(
    "execute_waypoints", 
    std::bind(&RobotController::exec_waypoints_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    exec_srv_ser_cbg_);

  stop_exec_srv_ = create_service<Trigger>(
    "stop_execution", 
    std::bind(&RobotController::stop_exec_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    exec_srv_ser_cbg_);

  // ============== Collision Objects Services ==============
  add_collision_obj_srv_ = create_service<AddCollisionObjects>(
    "add_collision_objects", 
    std::bind(&RobotController::add_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    col_obj_srv_ser_cbg_);

  remove_collision_obj_srv_ = create_service<RemoveCollisionObjects>(
    "remove_collision_objects", 
    std::bind(&RobotController::remove_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    col_obj_srv_ser_cbg_);

  apply_attached_collision_obj_srv_ = create_service<ApplyAttachedCollisionObjects>(
    "apply_attached_collision_objects", 
    std::bind(&RobotController::apply_attached_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    col_obj_srv_ser_cbg_);

  move_collision_obj_srv_ = create_service<MoveCollisionObjects>(
    "move_collision_objects", 
    std::bind(&RobotController::move_collision_obj_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    col_obj_srv_ser_cbg_);

  get_collision_obj_from_secne_srv_ = create_service<GetCollisionObjectsFromScene>(
    "get_collision_objects_from_sence", 
    std::bind(&RobotController::get_collision_obj_from_scene_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    get_col_obj_srv_ser_cbg_);
    
  // ============== Utilities Services ==============
  robot_speed_srv_ = create_service<RobotSpeed>(
    "robot_speed", 
    std::bind(&RobotController::robot_speed_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  get_joint_states_srv_ = create_service<GetJointStates>(
    "get_current_joint_states", 
    std::bind(&RobotController::get_joint_states_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  get_joint_limits_srv_ = create_service<GetJointLimits>(
    "get_joint_limits", 
    std::bind(&RobotController::get_joint_limits_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  get_pose_srv_ = create_service<GetPose>(
    "get_pose", 
    std::bind(&RobotController::get_pose_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  push_pose_arr_srv_ = create_service<PushPoseArray>(
    "push_pose_array", 
    std::bind(&RobotController::push_pose_array_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    exec_srv_ser_cbg_);

  clear_pose_arr_srv_ = create_service<Trigger>(
    "clear_pose_array", 
    std::bind(&RobotController::clear_pose_array_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    exec_srv_ser_cbg_);

  RCLCPP_INFO(get_logger(), "Robot Controller Node - initiated service servers");

  exe_trajectory_cli_ = rclcpp_action::create_client<ExecuteTrajectory>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "execute_trajectory",
    action_cli_cbg_);

  RCLCPP_INFO(get_logger(), "Robot Controller Node - initiated action clients");

  RCLCPP_INFO(get_logger(), "Robot Controller Node is up.");
}

void RobotController::pose_pub_cb(void)
{
  // move_group_->get_joint_limits();
  // std::optional<Pose> pose = move_group_->get_pose(eef_name_);

  // if (!pose.has_value())
  // {
  //   RCLCPP_ERROR(get_logger(), "Failed to get current pose for joint [%s]", eef_name_.c_str());
  //   return;
  // }

  // curr_pose_pub_->publish(std::move(pose.value()));
}

bool RobotController::are_poses_equal(
  const Pose &pose1, 
  const Pose &pose2, 
  double pos_thd, 
  double ori_thd)
{
  Eigen::Vector3d pos1(pose1.position.x, pose1.position.y, pose1.position.z);
  Eigen::Vector3d pos2(pose2.position.x, pose2.position.y, pose2.position.z);

  if ((pos1 - pos2).norm() > pos_thd) 
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
  return (q1.angularDistance(q2) < ori_thd);
}

void RobotController::process_waypoints(std::vector<Pose>& waypoints)
{
  if (waypoints.empty()) 
    return;
    
  std::vector<Pose> filtered_waypoints;
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

bool RobotController::exec_waypoints(
  const std::vector<Pose>& waypoints,
  const double speed,
  std::string *ret_msg)
{
  return exec_waypoints(waypoints, eef_step_.load(), jump_threshold_.load(), speed, ret_msg);
}

bool RobotController::exec_waypoints(
  const std::vector<Pose>& input_waypoints,
  const double eef_step,
  const double jump_threshold,
  const double speed,
  std::string *ret_msg)
{
  std::vector<Pose> waypoints;

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
  moveit_msgs::msg::MoveItErrorCodes err_codes;

  double fraction = move_group_->compute_cartesian_path(
    std::move(waypoints), 
    eef_step,
    jump_threshold, 
    trajectory, 
    true, 
    &err_codes);
    
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

  Float32 msg;
  msg.data = speed;
  speed_pub_->publish(msg);

  // trajectory.joint_trajectory.header.frame_id = std::to_string(speed);

  RCLCPP_WARN(get_logger(), "Scaling trajectory with robot speed: %.4f", speed / 100.0);
  move_group_->compute_timestamps_totg(trajectory, speed / 100.0, speed / 100.0);

  RCLCPP_WARN(get_logger(), "******** duration: %d.%d", 
    trajectory.joint_trajectory.points.back().time_from_start.sec, 
    trajectory.joint_trajectory.points.back().time_from_start.nanosec);

  moveit::core::MoveItErrorCode code = move_group_->execute(trajectory);

  if (code != 0 && code != moveit::core::MoveItErrorCode::SUCCESS)
  {
    const std::string err_msg = "Execute failed with error: " + std::to_string(code.val);
    *ret_msg = err_msg;

    RCLCPP_ERROR(get_logger(), "%s", err_msg.c_str());
    return false;
  } 

  return true; 
}

bool RobotController::exec_pushed_waypoints(std::string *ret_msg)
{
  if (pushed_waypoints_.empty())
  {
    RCLCPP_WARN(get_logger(), "No waypoints available for execution");
    return true;
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

void RobotController::testing_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  
  // auto goal_msg = ExecuteTrajectory::Goal();
  // goal_msg.trajectory = trajectory;

  // auto send_goal_options = rclcpp_action::Client<ExecuteTrajectory>::SendGoalOptions();
  // send_goal_options.feedback_callback = std::bind(&RobotController::feedback_cb, this, _1, _2);
  // auto goal_handle_future = exe_trajectory_cli_->async_send_goal(goal_msg, send_goal_options);

  // goal_handle_future.wait();
  
  // rclcpp_action::ClientGoalHandle<ExecuteTrajectory>::SharedPtr goal_handle = goal_handle_future.get();

  // auto result_future = exe_trajectory_cli_->async_get_result(goal_handle);
  // result_future.wait();
  
  response->success = true;
}


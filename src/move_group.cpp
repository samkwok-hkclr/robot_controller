#include "robot_controller/move_group.hpp"
#include "bspline/bspline.hpp"

MoveGroup::MoveGroup(std::shared_ptr<rclcpp::Node> node)
{
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  node_ = node;

  node_->declare_parameter<std::string>("robot_description", "");
  node_->declare_parameter<std::string>("robot_description_semantic", "");
  node_->declare_parameter<std::string>("robot_description_kinematics", "");

  RCLCPP_INFO(node_->get_logger(), "Move Group Controller Node is up.");
}

MoveGroup::~MoveGroup()
{

}

bool MoveGroup::config_robot(std::string description_path, std::string semantic_path, std::string kinematics_path)
{
  const rclcpp::Parameter description_param("robot_description", description_path);
  const rclcpp::Parameter semantic_param("robot_description_semantic", semantic_path);
  const rclcpp::Parameter kinematics_param("robot_description_kinematics", kinematics_path);

  node_->set_parameter(description_param);
  node_->set_parameter(semantic_param);
  node_->set_parameter(kinematics_param);
  
  return true;
}

bool MoveGroup::init_move_group(std::string group_name, std::string eef_name)
{
  move_group_ = std::make_unique<MoveGroupInterface>(node_, group_name);

  RCLCPP_INFO(node_->get_logger(), "group_name: %s", group_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "eef_name: %s", eef_name.c_str());

  if (!move_group_->setEndEffectorLink(eef_name))
  {
    RCLCPP_INFO(node_->get_logger(), "SetEndEffectorLink Failed.");
    return false;
  }

  planning_scene_ = std::make_unique<PlanningSceneInterface>();
  return true;
}

void MoveGroup::set_use_bspline(bool use, double step) 
{
  use_bspline_ = use;
  bspline_step_ = step;
}

geometry_msgs::msg::Pose MoveGroup::get_curr_pose(std::string joint_name)
{
  if (!move_group_)
  {
    RCLCPP_INFO(node_->get_logger(), "move_group_ does not exist.");
    return geometry_msgs::msg::Pose();
  }

  return move_group_->getCurrentPose(joint_name).pose;
}

sensor_msgs::msg::JointState MoveGroup::get_curr_joint_states()
{
  if (!move_group_)
  {
    RCLCPP_INFO(node_->get_logger(), "move_group_ does not exist.");
    return sensor_msgs::msg::JointState();
  }

  sensor_msgs::msg::JointState msg;
  msg.name = move_group_->getJointNames();
  msg.position = move_group_->getCurrentJointValues();

  return msg;
}

double MoveGroup::compute_cartesian_path(
  const std::vector<geometry_msgs::msg::Pose>& waypoints,
  const double eef_step, 
  const double jump_threshold,
  moveit_msgs::msg::RobotTrajectory& trajectory,
  bool avoid_collisions,
  moveit_msgs::msg::MoveItErrorCodes* error_code)
{
  if (waypoints.empty()) 
  {
    RCLCPP_ERROR(node_->get_logger(), "Waypoints vector is empty!");
    return 0.0;
  }

  if (!move_group_)
  {
    RCLCPP_INFO(node_->get_logger(), "move_group_ does not exist.");
    return 0.0;
  }

  double fraction = 0.0;

  auto compute_path = [&](const auto& wps) -> double {
    return move_group_->computeCartesianPath(wps, eef_step, jump_threshold, trajectory, avoid_collisions, error_code);
  };

  // ========== Non-B-spline Case ==========
  if (!use_bspline_)
  {
    fraction = compute_path(waypoints);
    return fraction;
  }

  // ========== B-spline Processing ==========
  auto generate_bspline = [&](bool set_point_flag) {
    Bspline bspliner(waypoints, set_point_flag);
    return bspliner.creatBspline(bspline_step_, waypoints);
  };

  // First attempt: setPointFlag = false
  auto waypoints_bspline = generate_bspline(false);
  fraction = compute_path(waypoints_bspline);

  // Second attempt: setPointFlag = true (if first failed)
  if (fraction < 1.0)
  {
    RCLCPP_WARN(node_->get_logger(), "First B-spline attempt failed (fraction=%.2f), trying setPointFlag=true", fraction);
    
    waypoints_bspline = generate_bspline(true);
    fraction = compute_path(waypoints_bspline);
  }

  // Final fallback: original waypoints (if both B-spline attempts failed)
  if (fraction < 1.0)
  {
    RCLCPP_WARN(node_->get_logger(), "B-spline completely failed (best fraction=%.2f), falling back to linear path", fraction);
    fraction = compute_path(waypoints);
  }

  return fraction;
}

bool MoveGroup::compute_joint_path(
  const std::vector<geometry_msgs::msg::Pose>& waypoints, 
  moveit_msgs::msg::RobotTrajectory& trajectory)
{
  if (waypoints.size() < 1)
  {
    RCLCPP_ERROR(node_->get_logger(), "Need 1 or more than 1 joint waypoints for computeJointPath!");
    return false;
  }   

  const moveit::core::JointModelGroup* joint_model_group = move_group_->getCurrentState()->getJointModelGroup(move_group_->getName());
  moveit_msgs::msg::RobotTrajectory whole_trajectory_msg;

  for (int cnt = 0; cnt < (int)waypoints.size(); cnt++)
  {
    // generate plan between every two joint waypoint
    geometry_msgs::msg::Pose goal_pose = waypoints[cnt];
    move_group_->setPoseTarget(goal_pose);

    moveit::planning_interface::MoveGroupInterface::Plan partial_plan;
    if (move_group_->plan(partial_plan) != moveit::core::MoveItErrorCode::SUCCESS)
    { 
      RCLCPP_ERROR(node_->get_logger(), "Failed to compute joint path from current pose to %s", "goal_pose [FIXME in move_group.cpp]");
      move_group_->setStartStateToCurrentState();
      return false;
    }

    // trajectory fusion
    whole_trajectory_msg.joint_trajectory.joint_names = partial_plan.trajectory_.joint_trajectory.joint_names;
    whole_trajectory_msg.joint_trajectory.points.insert(
        whole_trajectory_msg.joint_trajectory.points.end(), 
        partial_plan.trajectory_.joint_trajectory.points.begin(), 
        partial_plan.trajectory_.joint_trajectory.points.end());
    
    if (cnt != (int)waypoints.size() - 1) 
    {
      moveit::core::RobotState start_state(*move_group_->getCurrentState());
      if(!start_state.setFromIK(joint_model_group, goal_pose, move_group_->getEndEffectorLink()))
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to set the start state");
        move_group_->setStartStateToCurrentState();
        return false;
      }
      move_group_->setStartState(start_state);
    } 
  }

  move_group_->setStartStateToCurrentState();

  // whole trajectory replanning under time optmization strategy
  robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
  rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), whole_trajectory_msg);
    
  // trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // if (!iptp.computeTimeStamps(rt, 1.0, 1.0))
  // { 
  //   RCLCPP_ERROR(node_->get_logger(), "Computing time stamps for JointPath failed");
  //   return false;
  // }
  // RCLCPP_INFO(node_->get_logger(), "**********iptp overal time: %f", rt.getDuration());

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  // Custom velocity & acceleration limits for some joints
  const double velocity_scaling = 1.0;
  const double acceleration_scaling = 1.0;
  if (!totg.computeTimeStamps(rt, velocity_scaling, acceleration_scaling))
  {
    RCLCPP_ERROR(node_->get_logger(), "Computing time stamps for JointPath failed");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "**********totg overal time: %f", rt.getDuration());

  // output result trajectory
  rt.getRobotTrajectoryMsg(trajectory);
  return true;
}

bool MoveGroup::compute_joint_path(
  const std::vector<std::vector<double>>& joint_waypoints, 
  moveit_msgs::msg::RobotTrajectory& trajectory)
{
  if (joint_waypoints.size() < 1)
  {
    RCLCPP_ERROR(node_->get_logger(), "Need 1 or more than 1 joint waypoints for computeJointPath!");
    return false;
  }    

  const moveit::core::JointModelGroup* joint_model_group = move_group_->getCurrentState()->getJointModelGroup(move_group_->getName());
  moveit_msgs::msg::RobotTrajectory whole_trajectory;

  for(int cnt = 0; cnt < (int)joint_waypoints.size(); cnt++)
  {
    // generate plan between every two joint waypoint
    moveit::core::RobotState start_state(*(move_group_->getCurrentState()));
    std::vector<double> start_joints;
    std::vector<double> goal_joints;

    if(cnt == 0)
    {
      goal_joints = joint_waypoints[0];
    }
    else
    {
      start_joints = joint_waypoints[cnt - 1];
      start_state.setJointGroupPositions(joint_model_group, start_joints);
      goal_joints = joint_waypoints[cnt];
    }

    move_group_->setStartState(start_state);
    move_group_->setJointValueTarget(goal_joints);
    moveit::planning_interface::MoveGroupInterface::Plan partial_plan;

    bool success = (move_group_->plan(partial_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!success)
    { 
      if (cnt == 0)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to compute joint path from current joints to %s", 
          vec_to_str(goal_joints).c_str());
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to compute joint path from %s to %s", 
          vec_to_str(start_joints).c_str(), vec_to_str(goal_joints).c_str());
      }

      move_group_->setStartStateToCurrentState();
      return false;
    }
    // trajectory fusion
    whole_trajectory.joint_trajectory.joint_names = partial_plan.trajectory_.joint_trajectory.joint_names;

    // add first point only for first planned trajectoy
    if (cnt == 0)
    {
      whole_trajectory.joint_trajectory.points.push_back(partial_plan.trajectory_.joint_trajectory.points[0]);
    }

    for (int i = 1; i < (int)partial_plan.trajectory_.joint_trajectory.points.size(); i++)
    {   
      // add points of planned trajectoies 
      whole_trajectory.joint_trajectory.points.push_back(partial_plan.trajectory_.joint_trajectory.points[i]);
    }
  }

    move_group_->setStartStateToCurrentState();

    // whole trajectory replanning under time optmization strategy
    robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
    rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), whole_trajectory);

    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    const double velocity_scaling = 1.0;
    const double acceleration_scaling = 1.0;
    bool success = iptp.computeTimeStamps(rt, velocity_scaling, acceleration_scaling);
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Computing time stamps for JointPath %s", success ? "SUCCEDED" : "FAILED");

    if (!success)
      return false;

    // output result trajectory
    rt.getRobotTrajectoryMsg(trajectory);
    return true;
}

moveit::core::MoveItErrorCode MoveGroup::execute_pose(
  const geometry_msgs::msg::Pose& pose)
{
  move_group_->setPoseTarget(pose);

  moveit::core::MoveItErrorCode code = move_group_->move();
  if (code != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(node_->get_logger(), "Failed with MoveItErrorCode: %d", code.val);
  }

  return code;
}

moveit::core::MoveItErrorCode MoveGroup::execute_joints(
  const std::vector<double>& joints)
{
  move_group_->setJointValueTarget(joints);
  move_group_->setMaxVelocityScalingFactor(0.3);

  moveit::core::MoveItErrorCode code = move_group_->move();
  if (code != moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_WARN(node_->get_logger(), "Failed with MoveItErrorCode: %d", code.val);
  }
  
  return code;
}

moveit::core::MoveItErrorCode MoveGroup::execute(
  const moveit_msgs::msg::RobotTrajectory& trajectory)
{
  std::unique_lock<std::mutex> l(execution_mtx_);

  if (!move_group_)
  {
    RCLCPP_INFO(node_->get_logger(), "move_group_ does not exist.");
    return moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN;
  }

  moveit::core::MoveItErrorCode code;
  code = move_group_->execute(trajectory);
  RCLCPP_INFO(node_->get_logger(), "executed a trajectory");

  return code;
}

void MoveGroup::stop()
{
  if (!move_group_)
  {
    RCLCPP_INFO(node_->get_logger(), "move_group_ does not exist.");
    return;
  }

  move_group_->stop();
  RCLCPP_INFO(node_->get_logger(), "stopped");
}

void MoveGroup::set_max_velocity_scaling_factor(double max_velocity_scaling_factor)
{
  if (max_velocity_scaling_factor > 1.0 || max_velocity_scaling_factor < 0.0) 
  {
    RCLCPP_WARN(node_->get_logger(), 
      "Invalid max_vel_scaling_factor: [%f], using default value [1.0].", 
      max_velocity_scaling_factor);

    max_velocity_scaling_factor = 1.0;
  }

  move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
}

void MoveGroup::set_max_acceleration_scaling_factor(double max_acceleration_scaling_factor)
{
  // Check if factor in [0 ,1], othrerwise it will be set to a default value of 1.0 internally
  if (max_acceleration_scaling_factor > 1.0 || max_acceleration_scaling_factor < 0.0) 
  {
    RCLCPP_WARN(node_->get_logger(), 
    "Invalid max_acc_scaling_factor: [%f], using default value [1.0].", 
    max_acceleration_scaling_factor);

    max_acceleration_scaling_factor = 1.0;
  }

  move_group_->setMaxVelocityScalingFactor(max_acceleration_scaling_factor);
}

bool MoveGroup::add_collision_objects(
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects,
  const std::vector<moveit_msgs::msg::ObjectColor>& object_colors)
{
  if (move_group_ == NULL || planning_scene_ == NULL)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed with error: No move group.");
    return false;
  }

  for (int i = 0; i < (int)collision_objects.size(); i++)
  {
    collision_objects[i].header.frame_id = move_group_->getPlanningFrame();
    RCLCPP_INFO(node_->get_logger(), "Add collision object: %s", collision_objects[i].id.c_str());
  }

  planning_scene_->addCollisionObjects(collision_objects, object_colors);

  return true;
}

bool MoveGroup::remove_collision_objects(const std::vector<std::string>& object_ids)
{
  if (move_group_ == NULL || planning_scene_ == NULL)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed with error: No move group.");
    return false;
  }
  
  // we want synchronous update of the scene as we use this func to remove the box to be picked
  // if async, plan **might** (by chance / when sys under heavy load) fail as the gripper collide with the box to be picked,
  // which should have been removed, but still stay at the scene
  std::vector<moveit_msgs::msg::CollisionObject> remove_objects;
  for (const auto& object_id : object_ids)
  {
    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id = object_id;        
    remove_object.operation = remove_object.REMOVE;
    remove_objects.push_back(remove_object);
  }

  planning_scene_->applyCollisionObjects(remove_objects); // this is sync
  //planning_scene_->removeCollisionObjects(object_ids); // this is async 
  
  return true;
}

bool MoveGroup::move_collision_object(std::string key, const geometry_msgs::msg::Pose& pose, bool is_mesh)
{
  if (move_group_ == NULL || planning_scene_ == NULL)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed with error: No move group.");
    return false;
  }

  moveit_msgs::msg::CollisionObject object;
  object.operation = object.MOVE;
  object.id = key;

  if (is_mesh)
  {
    object.mesh_poses.push_back(pose);
  } 
  else 
  {
    object.primitive_poses.push_back(pose);
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  objects.push_back(object);

  std::vector<moveit_msgs::msg::ObjectColor> colors;

  planning_scene_->addCollisionObjects(objects, colors);

  return true;
}

bool MoveGroup::apply_attached_collision_objects(
  const std::vector<moveit_msgs::msg::AttachedCollisionObject>& attached_collision_objects)
{
  if (move_group_ == NULL || planning_scene_ == NULL)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed with error: No move group.");
    return false;
  }

  planning_scene_->applyAttachedCollisionObjects(attached_collision_objects);

  return true;
}

std::map<std::string, moveit_msgs::msg::CollisionObject> MoveGroup::get_collision_objects_from_scene(
  const std::vector<std::string>& object_ids)
{
  if (move_group_ == NULL || planning_scene_ == NULL)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed with error: No move group.");
    return std::map<std::string, moveit_msgs::msg::CollisionObject>();
  }
  
  return planning_scene_->getObjects(object_ids); 
}

bool MoveGroup::compute_timestamps_totg(
  moveit_msgs::msg::RobotTrajectory& trajectory, 
  const double max_velocity_scaling_factor,
  const double max_acceleration_scaling_factor)
{
  // Post processing for planned trajectories
  robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), move_group_->getName());
  rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  // Custom velocity & acceleration limits for some joints
  if (!totg.computeTimeStamps(rt, max_velocity_scaling_factor, max_acceleration_scaling_factor))
  {
    RCLCPP_ERROR(node_->get_logger(), "Computing time stamps for JointPath failed");
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "********** totg overal time: %f", rt.getDuration());

  // output result trajectory
  rt.getRobotTrajectoryMsg(trajectory);

  return true;
}

std::string MoveGroup::vec_to_str(const std::vector<double>& vec)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < vec.size(); ++i) 
  {
    ss << vec[i];
    if (i < vec.size() - 1) 
      ss << ", ";
  }
  ss << "]";
  return ss.str();
}
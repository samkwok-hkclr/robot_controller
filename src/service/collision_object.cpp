#include "robot_controller/robot_ctlr.hpp"

void RobotController::add_collision_obj_cb(
  const std::shared_ptr<AddCollisionObjects::Request> request, 
  std::shared_ptr<AddCollisionObjects::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "Start to Handle %s", __FUNCTION__);

  const auto& colors = request->object_colors.empty() 
                      ? std::vector<moveit_msgs::msg::ObjectColor>() 
                      : request->object_colors;

  if (!move_group_->add_collision_objects(request->collision_objects, colors)) 
  {
    response->success = false;
    response->message = "add collision failed";
    return;
  }

  response->success = true;

  RCLCPP_INFO(get_logger(), "Handled a %s", __FUNCTION__);
} 

void RobotController::remove_collision_obj_cb(
  const std::shared_ptr<RemoveCollisionObjects::Request> request, 
  std::shared_ptr<RemoveCollisionObjects::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "Start to Handle %s", __FUNCTION__);

  if (!move_group_->remove_collision_objects(request->object_ids)) 
  {
    response->success = false;
    response->message = "remove collision failed";
   
    return;
  }

  response->success = true;
  
  RCLCPP_INFO(get_logger(), "Handled a %s", __FUNCTION__);
}

void RobotController::apply_attached_collision_obj_cb(
  const std::shared_ptr<ApplyAttachedCollisionObjects::Request> request, 
  std::shared_ptr<ApplyAttachedCollisionObjects::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "Start to Handle %s", __FUNCTION__);

  if (!move_group_->apply_attached_collision_objects(request->attached_collision_objects)) 
  {
    response->success = false;
    response->message = "apply attached collision failed";
    return;
  }

  response->success = true;

  RCLCPP_INFO(get_logger(), "Handled a %s", __FUNCTION__);
}

void RobotController::move_collision_obj_cb(
  const std::shared_ptr<MoveCollisionObjects::Request> request, 
  std::shared_ptr<MoveCollisionObjects::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "Start to Handle %s", __FUNCTION__);

  if (!move_group_->move_collision_object(request->key, request->pose, request->is_mesh)) 
  {
    response->success = false;
    response->message = "add collision failed";
    return;
  }

  response->success = true;

  RCLCPP_INFO(get_logger(), "Handled a %s", __FUNCTION__);
}

void RobotController::get_collision_obj_from_scene_cb(
  const std::shared_ptr<GetCollisionObjectsFromScene::Request> request, 
  std::shared_ptr<GetCollisionObjectsFromScene::Response> response)
{
  RCLCPP_DEBUG(get_logger(), "Start to Handle %s", __FUNCTION__);

  const auto& results = move_group_->get_collision_objects_from_scene(request->object_ids);
  
  if (results.empty()) 
  {
    response->success = true;
    response->message = "No collision objects found for the specified IDs";
    return;
  }

  response->id_map.reserve(results.size());

  // Transform map entries directly into response
  std::transform(results.begin(), results.end(), std::back_inserter(response->id_map),
    [](const auto& pair) {
      CollisionObjectMap msg;
      msg.object_id = pair.first,
      msg.collision_object = pair.second;
      return msg;
    });

  response->success = true;

  RCLCPP_DEBUG(get_logger(), "Handled a %s", __FUNCTION__);
}
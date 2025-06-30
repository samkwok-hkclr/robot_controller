#include "robot_controller/robot_ctlr_node.hpp"

void RobotControllerNode::add_collision_obj_cb(
  const std::shared_ptr<AddCollisionObjects::Request> request, 
  std::shared_ptr<AddCollisionObjects::Response> response)
{
  const auto& colors = request->object_colors.empty() 
                      ? std::vector<moveit_msgs::msg::ObjectColor>() 
                      : request->object_colors;

  if (move_group_->add_collision_objects(request->collision_objects, colors)) 
  {
    response->success = true;
    return;
  }

  response->success = false;
  response->message = "add collision failed";
}

void RobotControllerNode::remove_collision_obj_cb(
  const std::shared_ptr<RemoveCollisionObjects::Request> request, 
  std::shared_ptr<RemoveCollisionObjects::Response> response)
{
  if (move_group_->remove_collision_objects(request->object_ids)) 
  {
    response->success = true;
    return;
  }

  response->success = false;
  response->message = "remove collision failed";
}

void RobotControllerNode::apply_attached_collision_obj_cb(
  const std::shared_ptr<ApplyAttachedCollisionObjects::Request> request, 
  std::shared_ptr<ApplyAttachedCollisionObjects::Response> response)
{
  if (move_group_->apply_attached_collision_objects(request->attached_collision_objects)) 
  {
    response->success = true;
    return;
  }

  response->success = false;
  response->message = "add collision failed";
}

void RobotControllerNode::move_collision_obj_cb(
  const std::shared_ptr<MoveCollisionObjects::Request> request, 
  std::shared_ptr<MoveCollisionObjects::Response> response)
{
  if (move_group_->move_collision_object(request->key, request->pose, request->is_mesh)) 
  {
    response->success = true;
    return;
  }

  response->success = false;
  response->message = "add collision failed";
}

void RobotControllerNode::get_collision_obj_from_scene_cb(
  const std::shared_ptr<GetCollisionObjectsFromScene::Request> request, 
  std::shared_ptr<GetCollisionObjectsFromScene::Response> response)
{
  const auto& results = move_group_->get_collision_objects_from_scene(request->object_ids);

  if (results.empty()) 
  {
    response->success = false;
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
}
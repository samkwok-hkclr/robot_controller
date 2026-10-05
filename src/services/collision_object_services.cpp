#include "robot_controller/robot_controller_node.hpp"

#include <algorithm>
#include <iterator>

namespace robot_controller
{

void RobotControllerNode::cbAddCollisionObject(
  const std::shared_ptr<AddCollisionObjects::Request> request,
  std::shared_ptr<AddCollisionObjects::Response> response)
{
  if (!move_group_->addCollisionObjects(
        request->collision_objects, request->object_colors))
  {
    response->success = false;
    response->message = "add collision failed";
    return;
  }
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbRemoveCollisionObject(
  const std::shared_ptr<RemoveCollisionObjects::Request> request,
  std::shared_ptr<RemoveCollisionObjects::Response> response)
{
  if (!move_group_->removeCollisionObjects(request->object_ids)) {
    response->success = false;
    response->message = "remove collision failed";
    return;
  }
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbApplyAttachedCollisionObject(
  const std::shared_ptr<ApplyAttachedCollisionObjects::Request> request,
  std::shared_ptr<ApplyAttachedCollisionObjects::Response> response)
{
  if (!move_group_->applyAttachedCollisionObjects(request->attached_collision_objects)) {
    response->success = false;
    response->message = "apply attached collision failed";
    return;
  }
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbMoveCollisionObject(
  const std::shared_ptr<MoveCollisionObjects::Request> request,
  std::shared_ptr<MoveCollisionObjects::Response> response)
{
  if (!move_group_->moveCollisionObject(request->object_id, request->pose, request->is_mesh)) {
    response->success = false;
    response->message = "move collision failed";
    return;
  }
  response->success = true;
  response->message = "ok";
}

void RobotControllerNode::cbGetCollisionObjectsFromScene(
  const std::shared_ptr<GetCollisionObjectsFromScene::Request> request,
  std::shared_ptr<GetCollisionObjectsFromScene::Response> response)
{
  auto results = move_group_->getCollisionObjectsFromScene(request->object_ids);
  if (!results) {
    response->success = false;
    response->message = "Failed to query planning scene";
    return;
  }
  if (results->empty()) {
    response->success = true;
    response->message = "No collision objects found";
    return;
  }

  response->id_map.reserve(results->size());
  std::transform(
    std::make_move_iterator(results->begin()),
    std::make_move_iterator(results->end()),
    std::back_inserter(response->id_map),
    [](auto && pair) {
      robot_controller_msgs::msg::CollisionObjectMap entry;
      entry.object_id = pair.first;
      entry.collision_object = std::move(pair.second);
      return entry;
    });

  response->success = true;
  response->message = "ok";
}

}  // namespace robot_controller
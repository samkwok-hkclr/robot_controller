#include "robot_controller/robot_ctlr_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<RobotControllerNode>(options, exec);

  exec->add_node(node->get_node_base_interface());
  exec->spin();
    
  rclcpp::shutdown();
}

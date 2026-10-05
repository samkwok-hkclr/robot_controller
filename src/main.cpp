#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "robot_controller/robot_controller_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::NodeOptions options;
    // options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<robot_controller::RobotControllerNode>(options);
    if (!node->initialize()) {
      RCLCPP_FATAL(rclcpp::get_logger("robot_controller"), "Node initialization failed.");
      rclcpp::shutdown();
      return 1;
    }

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_controller"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
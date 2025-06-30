#pragma once

#include <string>
// #include "clr_common/CLRSceneObject.hpp"
// #include "clr_common/CLRLogger.h"

struct RobotInfo{
  int robot_id;
  std::string description;
  std::string robot_description_file;
  std::string robot_semantic_description_file;
  std::string kinematics_file;
  std::string ee_name;
  std::string planning_name;
  std::string move_group_task;
  double eef_step = 0.05;
  double jump_threshold = 0.0;
  bool use_bspline = false;
  double bspline_step = 0.1;

  void logConfig(){
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "--RobotInfo-----------------------------------------------------------");
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- id:                           %d", robot_id);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- description:                  %s", description.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- description_file:             %s", robot_description_file.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- semantic_description_file:    %s", robot_semantic_description_file.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- kinematics_file:              %s", kinematics_file.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- ee_name:                      %s", ee_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- planning_name:                %s", planning_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- move_group_task:              %s", move_group_task.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- eef_step:                     %f", eef_step);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- jump_threshold:               %f", jump_threshold);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- use_bspline:                  %d", use_bspline);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "- bspline_step:                 %f", bspline_step);
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "-----------------------------------------------------------------------");
  }
};

struct RobotConfig
{
  int id; // to be phased out, was the row ID in SQL table
  RobotInfo robot_info;
  std::vector<CLRSceneObject> collision_objects_;

  void logConfig(){
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "==RobotConfig===========================================================");
    robot_info.logConfig();

    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "--Collisions-----------------------------------------------------------");
    for(CLRSceneObject object : collision_objects_) 
    {
      object.printInfo();
    }
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "-----------------------------------------------------------------------");    
    RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), "========================================================================");
  };
};
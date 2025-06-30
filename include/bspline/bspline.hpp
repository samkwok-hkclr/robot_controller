#ifndef BSPLINE_HPP__
#define BSPLINE_HPP__

#pragma once

#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/pose_array.hpp"

using namespace std;

class Bspline //B样条曲线
{
public:
	Bspline(const std::vector<geometry_msgs::msg::Pose>& waypoints, bool setPointFlag);
	~Bspline();

	double BsplineBfunc(int i, int k, double uu); // 计算每个u和每个i对应的B样条
	vector<geometry_msgs::msg::Pose> creatBspline(double delta_u, vector<geometry_msgs::msg::Pose> waypoints); // 计算整个的B样条
	void setPoint(vector<geometry_msgs::msg::Point>& p); // 定义点
	vector<geometry_msgs::msg::Point> getPointfromPose(vector<geometry_msgs::msg::Pose> pose);
	geometry_msgs::msg::Quaternion setQ(geometry_msgs::msg::Pose pose);

private:
	int k; // 阶数
	int n; // 控制点数-1
	vector<double> u; //自变量
	double uBegin;
	double uEnd;

	vector<geometry_msgs::msg::Point> _p;
	vector<geometry_msgs::msg::Point> p; // 控制点
	vector<geometry_msgs::msg::Point> pTrack; // 轨迹点
	vector<geometry_msgs::msg::Quaternion> qTrack; 
	vector<geometry_msgs::msg::Pose> p_waypoints;
};

#endif // BSPLINE_HPP__
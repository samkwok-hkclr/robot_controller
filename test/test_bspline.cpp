#include <gtest/gtest.h>
#include "robot_controller/bspline.hpp"

TEST(Bspline, RequiresThreeWaypoints)
{
  std::vector<geometry_msgs::msg::Pose> two(2);
  EXPECT_THROW(robot_controller::Bspline(two, false), std::invalid_argument);
}

TEST(Bspline, PartitionOfUnity)
{
  // Three collinear waypoints.
  std::vector<geometry_msgs::msg::Pose> wps(3);
  wps[0].position.x = 0.0;
  wps[1].position.x = 0.5;
  wps[2].position.x = 1.0;

  robot_controller::Bspline spline(wps, false);
  auto track = spline.interpolate(0.1);

  EXPECT_GE(track.size(), 3u);
  // Curve should not overshoot the [0, 1] range.
  for (const auto & p : track) {
    EXPECT_GE(p.position.x, -1e-6);
    EXPECT_LE(p.position.x, 1.0 + 1e-6);
  }
}

TEST(Bspline, OrientationTrackMatchesPositionTrack)
{
  std::vector<geometry_msgs::msg::Pose> wps(4);
  for (std::size_t i = 0; i < wps.size(); ++i) {
    wps[i].position.x = static_cast<double>(i) * 0.1;
    wps[i].orientation.w = 1.0;  // identity
  }
  robot_controller::Bspline spline(wps, true);
  auto track = spline.interpolate(0.05);
  // All poses must have a unit quaternion.
  for (const auto & p : track) {
    const double n = std::sqrt(
      p.orientation.x * p.orientation.x +
      p.orientation.y * p.orientation.y +
      p.orientation.z * p.orientation.z +
      p.orientation.w * p.orientation.w);
    EXPECT_NEAR(n, 1.0, 1e-6);
  }
}
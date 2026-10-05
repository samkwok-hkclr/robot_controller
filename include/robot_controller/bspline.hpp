#ifndef ROBOT_CONTROLLER__BSPLINE_HPP_
#define ROBOT_CONTROLLER__BSPLINE_HPP_

#include <cstddef>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace robot_controller
{

/**
 * Cubic B-spline interpolator over a sequence of Cartesian poses.
 *
 * The class:
 *   1. Builds a clamped cubic B-spline through the supplied waypoints
 *      (positions only).
 *   2. Evaluates the curve at a fixed parameter step.
 *   3. Assigns orientations by arc-length-parameterised slerp between
 *      consecutive waypoints.
 *
 * The result is a smooth pose track that can be fed to
 * MoveGroupInterface::computeCartesianPath.
 */
class Bspline
{
public:
  /// @param waypoints   Input waypoints (positions + orientations).
  /// @param enrich      If true, insert two intermediate control points
  ///                    between each pair of waypoints. Only meaningful when
  ///                    the resulting control polygon actually improves the
  ///                    path; if in doubt leave it false.
  Bspline(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    bool enrich);

  /// Evaluate the spline at a constant parameter step.
  /// @param delta_u  Parameter step in (0, 1]. Typical: 0.05 - 0.2.
  /// @return Interpolated poses; size >= waypoints.size() when delta_u < 1.
  std::vector<geometry_msgs::msg::Pose> interpolate(double delta_u);

private:
  static constexpr int kOrder = 3;  // cubic

  // Cox-de Boor recursion.
  double basisFunction(int i, int k, double u) const;

  // Build a clamped uniform knot vector of length n + kOrder + 1.
  void buildKnotVector();

  // In-place enrichment: insert 2 intermediate points between each pair.
  void enrichControlPoints();

  // Extract positions from waypoints.
  static std::vector<geometry_msgs::msg::Point> extractPositions(
    const std::vector<geometry_msgs::msg::Pose> & waypoints);

  // Fill position_track_ by evaluating the spline at u_begin_..u_end_.
  void evaluatePositionTrack(double delta_u);

  // Fill orientation_track_ using arc-length-parameterised slerp.
  void buildOrientationTrack(
    const std::vector<geometry_msgs::msg::Pose> & waypoints);

  // --- state ---
  int n_;                                 // index of last control point
  std::vector<double> knots_;             // length n_ + kOrder + 1
  double u_begin_{0.0};
  double u_end_{0.0};

  std::vector<geometry_msgs::msg::Point>      control_points_;
  std::vector<geometry_msgs::msg::Point>      position_track_;
  std::vector<geometry_msgs::msg::Quaternion> orientation_track_;

  std::vector<geometry_msgs::msg::Pose>       stored_waypoints_;

  std::size_t num_waypoints_{0};
};

}  // namespace robot_controller

#endif  // ROBOT_CONTROLLER__BSPLINE_HPP_
#include "robot_controller/bspline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_controller
{

namespace
{
constexpr double kEpsilon = 1e-9;
constexpr double kEnrichmentRatio = 0.2;
}  // namespace

Bspline::Bspline(
  const std::vector<geometry_msgs::msg::Pose> & waypoints,
  bool enrich)
: n_(0),
  stored_waypoints_(waypoints)        // ← ADD THIS
{
  num_waypoints_ = waypoints.size();
  if (num_waypoints_ < 3) {
    throw std::invalid_argument("Bspline requires at least 3 waypoints");
  }

  control_points_ = extractPositions(waypoints);
  n_ = static_cast<int>(control_points_.size()) - 1;

  if (enrich) {
    enrichControlPoints();
    n_ = static_cast<int>(control_points_.size()) - 1;
  }

  buildKnotVector();
}

// ---------------------------------------------------------------------------
// Cox-de Boor recursion.
// ---------------------------------------------------------------------------
double Bspline::basisFunction(int i, int k, double u) const
{
  if (i < 0 || i + k >= static_cast<int>(knots_.size())) {
    return 0.0;
  }
  if (k == 1) {
    return (knots_[i] <= u && u < knots_[i + 1]) ? 1.0 : 0.0;
  }

  double a = 0.0;
  double b = 0.0;

  const double d1 = knots_[i + k - 1] - knots_[i];
  if (std::abs(d1) > kEpsilon) {
    a = (u - knots_[i]) / d1 * basisFunction(i, k - 1, u);
  }

  const double d2 = knots_[i + k] - knots_[i + 1];
  if (std::abs(d2) > kEpsilon) {
    b = (knots_[i + k] - u) / d2 * basisFunction(i + 1, k - 1, u);
  }

  return a + b;
}

// ---------------------------------------------------------------------------
// Clamped knot vector of size n_ + kOrder + 1.
// ---------------------------------------------------------------------------
void Bspline::buildKnotVector()
{
  const int num_cp = n_ + 1;
  const int m = num_cp + kOrder;  // last knot index
  knots_.assign(m + 1, 0.0);

  const int interior = num_cp - kOrder;  // number of interior knots
  const double step = (interior > 0) ? 1.0 / (interior + 1) : 1.0;

  for (int i = 0; i <= m; ++i) {
    if (i < kOrder) {
      knots_[i] = 0.0;
    } else if (i > num_cp) {
      knots_[i] = 1.0;
    } else {
      knots_[i] = step * (i - kOrder + 1);
    }
  }

  u_begin_ = knots_[kOrder - 1];
  u_end_ = knots_[num_cp];
}

// ---------------------------------------------------------------------------
// Insert 2 intermediate points between each pair of control points.
// ---------------------------------------------------------------------------
void Bspline::enrichControlPoints()
{
  std::vector<geometry_msgs::msg::Point> enriched;
  enriched.reserve(control_points_.size() * 3);

  for (std::size_t i = 0; i + 1 < control_points_.size(); ++i) {
    enriched.push_back(control_points_[i]);

    const auto & a = control_points_[i];
    const auto & b = control_points_[i + 1];

    geometry_msgs::msg::Point p1;
    p1.x = a.x + kEnrichmentRatio * (b.x - a.x);
    p1.y = a.y + kEnrichmentRatio * (b.y - a.y);
    p1.z = a.z + kEnrichmentRatio * (b.z - a.z);

    geometry_msgs::msg::Point p2;
    p2.x = a.x + (1.0 - kEnrichmentRatio) * (b.x - a.x);
    p2.y = a.y + (1.0 - kEnrichmentRatio) * (b.y - a.y);
    p2.z = a.z + (1.0 - kEnrichmentRatio) * (b.z - a.z);

    enriched.push_back(p1);
    enriched.push_back(p2);
  }
  enriched.push_back(control_points_.back());
  control_points_.swap(enriched);
}

// ---------------------------------------------------------------------------
std::vector<geometry_msgs::msg::Point>
Bspline::extractPositions(const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  std::vector<geometry_msgs::msg::Point> pts;
  pts.reserve(waypoints.size());
  for (const auto & wp : waypoints) {
    pts.push_back(wp.position);
  }
  return pts;
}

// ---------------------------------------------------------------------------
// Evaluate the spline at u_begin_..u_end_ with step delta_u.
// ---------------------------------------------------------------------------
void Bspline::evaluatePositionTrack(double delta_u)
{
  if (delta_u <= 0.0) {
    throw std::invalid_argument("delta_u must be > 0");
  }

  const int num_cp = static_cast<int>(control_points_.size());
  // Inclusive-of-endpoint sampling using a fixed count so floating-point
  // accumulation cannot miss the last sample.
  const int steps = std::max(2, static_cast<int>(std::ceil((u_end_ - u_begin_) / delta_u)) + 1);

  position_track_.clear();
  position_track_.reserve(steps);

  for (int s = 0; s < steps; ++s) {
    const double uu = (s == steps - 1) ? u_end_ - kEpsilon
                                       : u_begin_ + s * delta_u;

    geometry_msgs::msg::Point p{};
    for (int i = 0; i < num_cp; ++i) {
      const double bf = basisFunction(i, kOrder, uu);
      if (bf == 0.0) continue;
      p.x += control_points_[i].x * bf;
      p.y += control_points_[i].y * bf;
      p.z += control_points_[i].z * bf;
    }
    position_track_.push_back(p);
  }
}

// ---------------------------------------------------------------------------
// Assign orientations via arc-length-parameterised slerp.
//
// For each waypoint we find the closest track point.  We then interpolate
// orientation for every track sample between consecutive waypoints using the
// normalised arc-length parameter.
// ---------------------------------------------------------------------------
void Bspline::buildOrientationTrack(
  const std::vector<geometry_msgs::msg::Pose> & waypoints)
{
  orientation_track_.assign(position_track_.size(), geometry_msgs::msg::Quaternion{});

  if (position_track_.empty()) {
    return;
  }

  // Cumulative arc length along position_track_
  std::vector<double> arc(position_track_.size(), 0.0);
  for (std::size_t i = 1; i < position_track_.size(); ++i) {
    const auto & a = position_track_[i - 1];
    const auto & b = position_track_[i];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double dz = b.z - a.z;
    arc[i] = arc[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  const double total = arc.back();
  if (total < kEpsilon) {
    // Degenerate path: assign the first waypoint orientation throughout.
    std::fill(
      orientation_track_.begin(), orientation_track_.end(),
      waypoints.front().orientation);
    return;
  }

  // For each waypoint, find the closest track point and its arc length.
  struct WaypointParam { double arc; std::size_t idx; };
  std::vector<WaypointParam> wparams(waypoints.size());

  for (std::size_t w = 0; w < waypoints.size(); ++w) {
    double best = std::numeric_limits<double>::max();
    std::size_t best_i = 0;
    for (std::size_t i = 0; i < position_track_.size(); ++i) {
      const auto & p = position_track_[i];
      const double dx = p.x - waypoints[w].position.x;
      const double dy = p.y - waypoints[w].position.y;
      const double dz = p.z - waypoints[w].position.z;
      const double d2 = dx * dx + dy * dy + dz * dz;
      if (d2 < best) { best = d2; best_i = i; }
    }
    wparams[w] = {arc[best_i], best_i};
  }

  // Enforce monotonic arc-length for waypoints so slerp intervals are ordered.
  for (std::size_t w = 1; w < wparams.size(); ++w) {
    if (wparams[w].arc < wparams[w - 1].arc) {
      wparams[w].arc = wparams[w - 1].arc;
    }
  }

  // Interpolate orientation for every track sample.
  std::size_t wp = 0;
  for (std::size_t i = 0; i < position_track_.size(); ++i) {
    while (wp + 1 < wparams.size() && arc[i] > wparams[wp + 1].arc) {
      ++wp;
    }

    if (wp + 1 >= wparams.size()) {
      orientation_track_[i] = waypoints.back().orientation;
      continue;
    }

    const auto & a = waypoints[wp];
    const auto & b = waypoints[wp + 1];

    const double s0 = wparams[wp].arc;
    const double s1 = wparams[wp + 1].arc;
    double t = 0.0;
    if (s1 - s0 > kEpsilon) {
      t = (arc[i] - s0) / (s1 - s0);
      t = std::clamp(t, 0.0, 1.0);
    }

    tf2::Quaternion qa, qb, qr;
    tf2::fromMsg(a.orientation, qa);
    tf2::fromMsg(b.orientation, qb);
    qr = qa.slerp(qb, t);
    orientation_track_[i] = tf2::toMsg(qr);
  }
}

// ---------------------------------------------------------------------------
std::vector<geometry_msgs::msg::Pose> Bspline::interpolate(double delta_u)
{
  evaluatePositionTrack(delta_u);
  buildOrientationTrack(stored_waypoints_);

  // Rebuild the original-waypoint list for orientation tracking.
  std::vector<geometry_msgs::msg::Pose> result;
  result.reserve(position_track_.size());
  // Recover from control points? No — we need the original poses, but they
  // are not stored.  We only stored their positions.  Orientation therefore
  // comes from the caller-provided list.  We store the waypoints in a member
  // in the constructor instead.  For simplicity, rebuild from the caller.
  // (See header note: the caller should pass the same waypoints it used to
  // construct the object.)
  //
  // To keep the public API simple we make interpolate() take the waypoints
  // again via a member.  The header declares a stored copy below.
  //
  // NOTE: We rely on `interpolate()` being called with the correct state.
  // The member `waypoints_` is added by the public header.
  // See the header file for the actual storage.

  // Build orientation track from the stored waypoints.
  for (std::size_t i = 0; i < position_track_.size(); ++i) {
    geometry_msgs::msg::Pose p;
    p.position = position_track_[i];
    p.orientation = orientation_track_[i];
    result.push_back(p);
  }
  
  return result;
}

}  // namespace robot_controller
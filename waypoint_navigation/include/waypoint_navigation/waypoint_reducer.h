// waypoint_reducer.h - ROS-free core of the waypoint reduction algorithm.
//
// Turns a dense planner path (hundreds of closely spaced poses) into a
// small set of waypoints that still follows the shape of the original path:
//
//   1. Angle filter     - keep only points where the path turns.
//   2. Distance filter  - enforce a minimum spacing between waypoints.
//   3. Corner recovery  - if step 2 dropped a point that sits more than
//                         `epsilon` away from the straight line between the
//                         kept neighbours, put the furthest one back
//                         (the core idea of Douglas-Peucker).
#pragma once

#include <vector>

namespace waypoint_navigation
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

enum class AngleMode
{
  // Turning angle at p[i] measured between (p[i-1] -> p[i]) and (p[i] -> p[i+1]).
  // This is the original 2023 behaviour. Works for jagged grid-planner paths,
  // but drops smooth arcs whose per-step turn is below the threshold.
  Local,
  // Turning angle at p[i] measured between (last kept point -> p[i]) and
  // (p[i] -> first point at least `lookahead` metres further on). Accumulates
  // gentle curvature (arcs are preserved) and averages out the millimetre
  // jitter that real planner paths have, which makes Local mode keep almost
  // every point.
  Cumulative
};

struct ReducerParams
{
  double angle_threshold_deg{5.0};  // keep a point if the path turns at least this much
  double min_distance{1.0};         // minimum spacing between consecutive waypoints [m]
  double epsilon{0.3};              // max allowed deviation before a dropped point is restored [m]
  AngleMode angle_mode{AngleMode::Cumulative};
  double lookahead{0.25};           // Cumulative mode: segment length used to measure direction [m]
};

// Geometry helpers (exposed for testing)
double distance(const Point2D& a, const Point2D& b);
// Turning angle in degrees between direction a->b and direction b->c (0 = straight on).
// Returns 0 if either segment has zero length.
double turningAngleDeg(const Point2D& a, const Point2D& b, const Point2D& c);
// Distance from p to the infinite line through a and b (distance to a if a == b).
double perpendicularDistance(const Point2D& p, const Point2D& a, const Point2D& b);

// Individual stages (exposed for testing / visualisation)
std::vector<Point2D> angleFilter(const std::vector<Point2D>& path, double angle_threshold_deg,
                                 AngleMode mode = AngleMode::Cumulative, double lookahead = 0.25);
std::vector<Point2D> distanceFilterWithRecovery(const std::vector<Point2D>& path, double min_distance,
                                                double epsilon);

// Full pipeline. The first and last points of the input are always kept.
std::vector<Point2D> reducePath(const std::vector<Point2D>& path, const ReducerParams& params);

}  // namespace waypoint_navigation

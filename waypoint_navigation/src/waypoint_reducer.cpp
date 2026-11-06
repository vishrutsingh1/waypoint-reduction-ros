#include "waypoint_navigation/waypoint_reducer.h"

#include <algorithm>
#include <cmath>

namespace waypoint_navigation
{
namespace
{
constexpr double kEps = 1e-9;

bool samePoint(const Point2D& a, const Point2D& b)
{
  return distance(a, b) < kEps;
}

// Restore the rejected point furthest from line (start, end), if it deviates by >= epsilon.
void recoverCorner(const std::vector<Point2D>& rejected, const Point2D& start, const Point2D& end,
                   double epsilon, std::vector<Point2D>& out)
{
  double best = -1.0;
  const Point2D* best_pt = nullptr;
  for (const auto& r : rejected)
  {
    const double d = perpendicularDistance(r, start, end);
    if (d >= epsilon && d > best)
    {
      best = d;
      best_pt = &r;
    }
  }
  if (best_pt)
    out.push_back(*best_pt);
}
}  // namespace

double distance(const Point2D& a, const Point2D& b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double turningAngleDeg(const Point2D& a, const Point2D& b, const Point2D& c)
{
  const double v1x = b.x - a.x, v1y = b.y - a.y;
  const double v2x = c.x - b.x, v2y = c.y - b.y;
  const double m1 = std::hypot(v1x, v1y);
  const double m2 = std::hypot(v2x, v2y);
  if (m1 < kEps || m2 < kEps)
    return 0.0;
  // atan2(|cross|, dot) is numerically stable near 0 and 180 degrees (no acos domain errors)
  const double cross = v1x * v2y - v1y * v2x;
  const double dot = v1x * v2x + v1y * v2y;
  return std::atan2(std::fabs(cross), dot) * 180.0 / M_PI;
}

double perpendicularDistance(const Point2D& p, const Point2D& a, const Point2D& b)
{
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len = std::hypot(dx, dy);
  if (len < kEps)
    return distance(p, a);
  // |cross(b - a, p - a)| / |b - a|
  return std::fabs(dx * (p.y - a.y) - dy * (p.x - a.x)) / len;
}

std::vector<Point2D> angleFilter(const std::vector<Point2D>& path, double angle_threshold_deg, AngleMode mode,
                                 double lookahead)
{
  if (path.size() <= 2)
    return path;

  std::vector<Point2D> out{ path.front() };
  size_t ahead = 1;  // index of the lookahead point (monotonic)
  for (size_t i = 1; i + 1 < path.size(); ++i)
  {
    if (mode == AngleMode::Local)
    {
      if (turningAngleDeg(path[i - 1], path[i], path[i + 1]) >= angle_threshold_deg)
        out.push_back(path[i]);
      continue;
    }
    // Cumulative: direction since the last kept point vs direction over the next `lookahead` metres
    if (distance(out.back(), path[i]) < lookahead)
      continue;
    ahead = std::max(ahead, i + 1);
    while (ahead + 1 < path.size() && distance(path[i], path[ahead]) < lookahead)
      ++ahead;
    if (turningAngleDeg(out.back(), path[i], path[ahead]) >= angle_threshold_deg)
    {
      // The turn happens somewhere in [i, ahead): keep the point furthest from the local chord
      // (behind -> ahead), i.e. the actual corner, rather than the first point that noticed the turn.
      size_t behind = i;
      while (behind > 0 && distance(path[behind], path[i]) < lookahead)
        --behind;
      size_t corner = i;
      double best = -1.0;
      for (size_t k = i; k < ahead; ++k)
      {
        const double d = perpendicularDistance(path[k], path[behind], path[ahead]);
        if (d > best)
        {
          best = d;
          corner = k;
        }
      }
      out.push_back(path[corner]);
      i = corner;
    }
  }
  if (!samePoint(out.back(), path.back()))
    out.push_back(path.back());
  return out;
}

std::vector<Point2D> distanceFilterWithRecovery(const std::vector<Point2D>& path, double min_distance,
                                                double epsilon)
{
  if (path.size() <= 2)
    return path;

  std::vector<Point2D> out{ path.front() };
  std::vector<Point2D> rejected;

  for (size_t i = 1; i < path.size(); ++i)
  {
    const bool is_goal = (i + 1 == path.size());
    if (distance(out.back(), path[i]) >= min_distance || is_goal)
    {
      // Accept path[i]; first check whether a dropped point in between was a real corner
      recoverCorner(rejected, out.back(), path[i], epsilon, out);
      rejected.clear();
      if (!samePoint(out.back(), path[i]))
        out.push_back(path[i]);
    }
    else
    {
      rejected.push_back(path[i]);
    }
  }
  return out;
}

std::vector<Point2D> reducePath(const std::vector<Point2D>& path, const ReducerParams& params)
{
  return distanceFilterWithRecovery(angleFilter(path, params.angle_threshold_deg, params.angle_mode, params.lookahead),
                                    params.min_distance, params.epsilon);
}

}  // namespace waypoint_navigation

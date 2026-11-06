#include <gtest/gtest.h>

#include <cmath>

#include "waypoint_navigation/waypoint_reducer.h"

using namespace waypoint_navigation;

namespace
{
std::vector<Point2D> line(Point2D a, Point2D b, double step)
{
  std::vector<Point2D> out;
  const int n = static_cast<int>(std::round(distance(a, b) / step));
  for (int i = 0; i < n; ++i)
    out.push_back({ a.x + (b.x - a.x) * i / n, a.y + (b.y - a.y) * i / n });
  return out;
}
}  // namespace

TEST(Geometry, PerpendicularDistanceIsInMetres)
{
  // Long segment: the 2023 version divided by |d|^2 and got this wrong
  EXPECT_NEAR(perpendicularDistance({ 5, 2 }, { 0, 0 }, { 10, 0 }), 2.0, 1e-9);
  EXPECT_NEAR(perpendicularDistance({ 3, 4 }, { 0, 0 }, { 0, 0 }), 5.0, 1e-9);
}

TEST(Geometry, TurningAngle)
{
  EXPECT_NEAR(turningAngleDeg({ 0, 0 }, { 1, 0 }, { 2, 0 }), 0.0, 1e-9);
  EXPECT_NEAR(turningAngleDeg({ 0, 0 }, { 1, 0 }, { 1, 1 }), 90.0, 1e-9);
  EXPECT_NEAR(turningAngleDeg({ 0, 0 }, { 1, 0 }, { 1, 0 }), 0.0, 1e-9);  // duplicate point, no NaN
}

TEST(Reducer, LShapeKeepsStartCornerGoal)
{
  auto path = line({ 0, 0 }, { 5, 0 }, 0.05);
  auto leg2 = line({ 5, 0 }, { 5, 5 }, 0.05);
  path.insert(path.end(), leg2.begin(), leg2.end());
  path.push_back({ 5, 5 });

  const auto out = reducePath(path, ReducerParams{});
  ASSERT_EQ(out.size(), 3u);
  EXPECT_NEAR(out[1].x, 5.0, 1e-6);
  EXPECT_NEAR(out[1].y, 0.0, 1e-6);
  EXPECT_NEAR(out.back().y, 5.0, 1e-6);
}

TEST(Reducer, GoalSharingAnAxisIsKept)
{
  // 2023 bug: '&&' instead of '||' dropped a goal that shared x or y with the last waypoint
  std::vector<Point2D> path = { { 0, 0 }, { 2, 0 }, { 2, 2 }, { 2, 2.3 } };
  const auto out = reducePath(path, ReducerParams{});
  EXPECT_NEAR(out.back().x, 2.0, 1e-9);
  EXPECT_NEAR(out.back().y, 2.3, 1e-9);
}

TEST(Reducer, CumulativeModeIgnoresPlannerJitter)
{
  // 10 m straight line with +-5 mm zig-zag every 5 cm: per-step angles are ~11 deg
  std::vector<Point2D> path;
  for (int i = 0; i <= 200; ++i)
    path.push_back({ i * 0.05, (i % 2 ? 0.005 : -0.005) });

  EXPECT_TRUE(angleFilter(path, 5.0, AngleMode::Cumulative).size() <= 3u);
  EXPECT_TRUE(angleFilter(path, 5.0, AngleMode::Local).size() > 100u);  // original behaviour
}

TEST(Reducer, DegenerateInputs)
{
  EXPECT_TRUE(reducePath({}, ReducerParams{}).empty());
  EXPECT_EQ(reducePath({ { 1, 1 } }, ReducerParams{}).size(), 1u);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

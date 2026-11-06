// waypoint_server.cpp - ROS 1 service wrapper around the waypoint reducer.
//
// Service  ~waypoints (nav_msgs/GetPlan): same request as move_base/make_plan;
//          returns the reduced waypoint list instead of the dense path.
// Publishes ~waypoints_path (nav_msgs/Path) and ~waypoints_markers (visualization_msgs/Marker) for RViz.
#include <cmath>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include "waypoint_navigation/waypoint_reducer.h"

namespace wn = waypoint_navigation;

class WaypointServer
{
public:
  explicit WaypointServer(ros::NodeHandle& pnh) : pnh_(pnh)
  {
    std::string planner_service, mode;
    pnh_.param<std::string>("planner_service", planner_service, "move_base/make_plan");
    pnh_.param("angle_threshold_deg", params_.angle_threshold_deg, 5.0);
    pnh_.param("min_distance", params_.min_distance, 1.0);
    pnh_.param("epsilon", params_.epsilon, 0.3);
    pnh_.param<std::string>("angle_mode", mode, "cumulative");
    params_.angle_mode = (mode == "local") ? wn::AngleMode::Local : wn::AngleMode::Cumulative;

    planner_ = ros::NodeHandle().serviceClient<nav_msgs::GetPlan>(planner_service);
    server_ = pnh_.advertiseService("waypoints", &WaypointServer::callback, this);
    path_pub_ = pnh_.advertise<nav_msgs::Path>("waypoints_path", 1, true);
    marker_pub_ = pnh_.advertise<visualization_msgs::Marker>("waypoints_markers", 1, true);

    ROS_INFO("Waypoint server: planner=%s angle=%.1fdeg min_dist=%.2fm eps=%.2fm mode=%s",
             planner_.getService().c_str(), params_.angle_threshold_deg, params_.min_distance,
             params_.epsilon, mode.c_str());
  }

private:
  bool callback(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& res)
  {
    nav_msgs::GetPlan plan;
    plan.request = req;
    if (!planner_.call(plan) || plan.response.plan.poses.empty())
    {
      ROS_ERROR("Planner %s returned no path", planner_.getService().c_str());
      return false;
    }
    const auto& dense = plan.response.plan;

    std::vector<wn::Point2D> pts;
    pts.reserve(dense.poses.size());
    for (const auto& p : dense.poses)
      pts.push_back({ p.pose.position.x, p.pose.position.y });

    const auto reduced = wn::reducePath(pts, params_);

    res.plan.header.frame_id = dense.header.frame_id.empty() ? req.start.header.frame_id : dense.header.frame_id;
    res.plan.header.stamp = ros::Time::now();
    for (size_t i = 0; i < reduced.size(); ++i)
    {
      geometry_msgs::PoseStamped ps;
      ps.header = res.plan.header;
      ps.pose.position.x = reduced[i].x;
      ps.pose.position.y = reduced[i].y;
      if (i + 1 < reduced.size())
      {
        // face the next waypoint
        tf2::Quaternion q;
        q.setRPY(0, 0, std::atan2(reduced[i + 1].y - reduced[i].y, reduced[i + 1].x - reduced[i].x));
        ps.pose.orientation = tf2::toMsg(q);
      }
      else
      {
        ps.pose.orientation = req.goal.pose.orientation;  // keep requested goal heading
      }
      res.plan.poses.push_back(ps);
    }

    publishVisualisation(res.plan);
    ROS_INFO("Reduced %zu path poses to %zu waypoints", dense.poses.size(), reduced.size());
    return true;
  }

  void publishVisualisation(const nav_msgs::Path& path)
  {
    path_pub_.publish(path);

    visualization_msgs::Marker m;
    m.header = path.header;
    m.ns = "waypoints";
    m.id = 0;
    m.type = visualization_msgs::Marker::POINTS;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = 0.15;
    m.color.b = 1.0f;
    m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;
    for (const auto& p : path.poses)
      m.points.push_back(p.pose.position);
    marker_pub_.publish(m);
  }

  ros::NodeHandle pnh_;
  wn::ReducerParams params_;
  ros::ServiceClient planner_;
  ros::ServiceServer server_;
  ros::Publisher path_pub_, marker_pub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_server");
  ros::NodeHandle pnh("~");
  WaypointServer server(pnh);
  ros::spin();
  return 0;
}

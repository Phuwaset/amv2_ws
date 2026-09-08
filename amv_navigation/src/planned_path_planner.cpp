#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.h>

namespace amv_navigation
{

class PlannedPathPlanner : public nav_core::BaseGlobalPlanner
{
public:
  PlannedPathPlanner() : costmap_ros_(NULL), have_path_(false), initialized_(false), max_match_dist_(1.0)
  {
  }

  PlannedPathPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    initialize(name, costmap_ros);
  }

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  {
    if (!initialized_)
    {
      costmap_ros_ = costmap_ros;
      ros::NodeHandle nh("~");

      double match_dist = 1.0;
      nh.param("planned_path_max_match_dist", match_dist, 1.0);
      max_match_dist_ = match_dist;

      path_sub_ = nh.subscribe("/planned_path", 1, &PlannedPathPlanner::pathCallback, this);

      navfn_.initialize("navfn_fallback", costmap_ros_);
      initialized_ = true;
      ROS_INFO("PlannedPathPlanner initialized (follows /planned_path, fallback NavfnROS).");
    }
  }

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan)
  {
    if (!have_path_ || latest_path_.poses.empty())
    {
      ROS_WARN_THROTTLE(2.0, "PlannedPathPlanner: no /planned_path yet -> fallback NavfnROS.");
      return navfn_.makePlan(start, goal, plan);
    }

    int start_idx = findNearestIndex(start);
    int goal_idx = findNearestIndex(goal);

    if (start_idx < 0 || goal_idx < 0)
    {
      ROS_WARN_THROTTLE(2.0, "PlannedPathPlanner: start/goal not on /planned_path (match > %.2f m) -> fallback NavfnROS.", max_match_dist_);
      return navfn_.makePlan(start, goal, plan);
    }

    plan.clear();
    if (start_idx <= goal_idx)
    {
      for (int i = start_idx; i <= goal_idx; ++i)
        plan.push_back(latest_path_.poses[i]);
    }
    else
    {
      for (int i = start_idx; i >= goal_idx; --i)
        plan.push_back(latest_path_.poses[i]);
    }

    plan.front() = start;
    plan.back() = goal;

    ROS_INFO("PlannedPathPlanner: returning %lu poses from /planned_path (start_idx=%d goal_idx=%d).", plan.size(), start_idx, goal_idx);
    return true;
  }

private:
  void pathCallback(const nav_msgs::Path& msg)
  {
    latest_path_ = msg;
    have_path_ = !msg.poses.empty();
    ROS_INFO("PlannedPathPlanner: /planned_path updated (%lu poses).", msg.poses.size());
  }

  int findNearestIndex(const geometry_msgs::PoseStamped& pose)
  {
    int best = -1;
    double best_d = max_match_dist_;
    for (size_t i = 0; i < latest_path_.poses.size(); ++i)
    {
      const geometry_msgs::Point& p = latest_path_.poses[i].pose.position;
      double d = std::hypot(p.x - pose.pose.position.x, p.y - pose.pose.position.y);
      if (d < best_d)
      {
        best_d = d;
        best = static_cast<int>(i);
      }
    }
    return best;
  }

  costmap_2d::Costmap2DROS* costmap_ros_;
  nav_msgs::Path latest_path_;
  bool have_path_;
  bool initialized_;
  double max_match_dist_;
  navfn::NavfnROS navfn_;
  ros::Subscriber path_sub_;
};

}  // namespace amv_navigation

PLUGINLIB_EXPORT_CLASS(amv_navigation::PlannedPathPlanner, nav_core::BaseGlobalPlanner)

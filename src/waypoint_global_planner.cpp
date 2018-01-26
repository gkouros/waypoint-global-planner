#include "waypoint_global_planner/waypoint_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>

PLUGINLIB_EXPORT_CLASS(waypoint_global_planner::WaypointGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace waypoint_global_planner
{

WaypointGlobalPlanner::WaypointGlobalPlanner() : costmap_ros_(NULL), initialized_(false), clear_waypoints_(false)
{
}


WaypointGlobalPlanner::WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}


WaypointGlobalPlanner::~WaypointGlobalPlanner()
{
}


void WaypointGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if(!initialized_)
  {
    // get the costmap
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = new base_local_planner::CostmapModel(*costmap_);

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~" + name);

    // load parameters
    pnh.param("epsilon", epsilon_, 1e-1);

    // initialize publishers and subscribers
    waypoint_sub_ = pnh.subscribe("/clicked_point", 10, &WaypointGlobalPlanner::waypointCallback, this);
    waypoint_marker_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("waypoints", 1);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    plan_pub_ = pnh.advertise<nav_msgs::Path>("global_plan", 1);

    initialized_ = true;
    ROS_INFO("Planner has been initialized");
  }
  else
  {
    ROS_WARN("This planner has already been initialized");
  }
}


bool WaypointGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start_pose,
  const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  generatePlanUsingWaypoints(start_pose, plan);
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses.insert(path.poses.end(), plan.begin(), plan.end());
  plan_pub_.publish(path);
  ROS_INFO("Published global plan");
  return true;
}


void WaypointGlobalPlanner::waypointCallback(const geometry_msgs::PointStampedConstPtr& waypoint)
{
  if (clear_waypoints_)
  {
    for (size_t i = 0; i < waypoint_markers_.markers.size(); i++)
      waypoint_markers_.markers[i].action = visualization_msgs::Marker::DELETE;

    waypoint_marker_pub_.publish(waypoint_markers_);
    waypoint_markers_.markers.clear();
    waypoints_.clear();
    clear_waypoints_ = false;
  }

  // add waypoint to the waypoint vector
  waypoints_.push_back(geometry_msgs::PoseStamped());
  waypoints_.back().header = waypoint->header;
  waypoints_.back().pose.position = waypoint->point;
  waypoints_.back().pose.orientation.w = 1.0;

  // create waypoint marker and add it to the marker vector
  visualization_msgs::Marker wp_marker;
  wp_marker.header = waypoint->header;
  wp_marker.ns = "/move_base/waypoint_global_planner";
  wp_marker.type = visualization_msgs::Marker::SPHERE;
  wp_marker.action = visualization_msgs::Marker::ADD;
  wp_marker.scale.x = 0.2;
  wp_marker.scale.y = 0.2;
  wp_marker.scale.z = 0.2;
  wp_marker.color.a = 1.0;
  wp_marker.color.r = 1.0;
  wp_marker.color.g = 0.0;
  wp_marker.color.b = 0.0;
  wp_marker.pose.position = waypoint->point;
  wp_marker.id = waypoint_markers_.markers.size() + 1;
  waypoint_markers_.markers.push_back(wp_marker);
  waypoint_marker_pub_.publish(waypoint_markers_);

  if (waypoints_.size() < 2)
    return;

  geometry_msgs::Pose *p1 = &(waypoints_.end()-2)->pose;
  geometry_msgs::Pose *p2 = &(waypoints_.end()-1)->pose;

  // calculate orientation of waypoints
  double yaw = atan2(p2->position.y - p1->position.y, p2->position.x - p1->position.x);
  p1->orientation = tf::createQuaternionMsgFromYaw(yaw);

  // calculate distance between latest two waypoints and check if it surpasses the threshold epsilon
  double dist = hypot(p1->position.x - p2->position.x, p1->position.y - p2->position.y);
  if (dist < epsilon_)
  {
    p2->orientation = p1->orientation;
    goal_pub_.publish(waypoints_.back());
    ROS_INFO("Published goal pose");
  }
}


void WaypointGlobalPlanner::generatePlanUsingWaypoints(const geometry_msgs::PoseStamped& start_pose,
  std::vector<geometry_msgs::PoseStamped>& plan)
{
  // generate plan
  plan.push_back(start_pose);
  plan.insert(plan.end(), waypoints_.begin(), waypoints_.end());

  // enable flag to clear waypoints and corresponding markers
  clear_waypoints_ = true;
}

}  // namespace waypoint_global_planner

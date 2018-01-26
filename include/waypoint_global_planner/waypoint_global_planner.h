#ifndef WAYPOINT_GLOBAL_PLANNER_H
#define WAYPOINT_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

namespace waypoint_global_planner
{

/**
 * @class CarrotPlanner
 * @brief Provides a simple global planner for producing boustrophedon paths without taking into account obstacles
 */
class WaypointGlobalPlanner : public nav_core::BaseGlobalPlanner
{
  public:
    /**
     * @brief Default Constructor
     */
    WaypointGlobalPlanner();

    /**
     * @brief Constructor for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    WaypointGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Default destructor
     */
    ~WaypointGlobalPlanner();

    /**
     * @brief Initialization function for the planner
     * @param name The name of this planner
     * @param costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    /**
     * @brief Given a goal pose in the world, compute a plan
     * @param start_pose The starting pose of the robot
     * @param goal The goal pose
     * @param plan The plan filled by the planner
     * @return True if a valid plan was found, false otherwise
     */
    bool makePlan(const geometry_msgs::PoseStamped& start_pose,
      const geometry_msgs::PoseStamped& goal,
      std::vector<geometry_msgs::PoseStamped>& plan);

    /**
     * @brief Waypoint callback
     * @param waypoint The received waypoint
     */
    void waypointCallback(const geometry_msgs::PointStampedConstPtr& waypoint);

    /**
     * @brief Waypoint callback
     * @param waypoint The received waypoint
     */
    void generatePlanUsingWaypoints(const geometry_msgs::PoseStamped& start_pose, std::vector<geometry_msgs::PoseStamped>& plan);


  private:
    bool initialized_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    base_local_planner::WorldModel* world_model_;

    // subscribers and publishers
    ros::Subscriber waypoint_sub_;
    ros::Publisher waypoint_marker_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher plan_pub_;

    // configuration parameters
    double epsilon_;

    // containers
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    visualization_msgs::MarkerArray waypoint_markers_;

    //flags
    bool clear_waypoints_;
};

}  // namespace waypoint_global_planner

#endif  // WAYPOINT_GLOBAL_PLANNER_H

#ifndef WAYPOINT_GLOBAL_PLANNER_H
#define WAYPOINT_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PointStamped.h>
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
    void waypointCallback(const geometry_msgs::PointStamped::ConstPtr& waypoint);

    /**
     * @brief External path callback
     * @param plan The received plan
     */
    void externalPathCallback(const nav_msgs::PathConstPtr& plan);

    /**
     * @brief Creates and publishes visualization markers from an input path
     * @param path The input path to be used for the creation of visualization markers
     */
    void createAndPublishMarkersFromPath(const std::vector<geometry_msgs::PoseStamped>& path);

    /**
     * @brief Interpolates a path (position and orientation) using a fixed number of points per meter
     * @param path The input path to be interpolated
     */
    void interpolatePath(nav_msgs::Path& path);

  private:
    bool initialized_;  //!< flag indicating the planner has been initialized
    costmap_2d::Costmap2DROS* costmap_ros_;  //!< costmap ros wrapper
    costmap_2d::Costmap2D* costmap_;  //!< costmap container
    base_local_planner::WorldModel* world_model_;  //!< world model

    // subscribers and publishers
    ros::Subscriber waypoint_sub_;  //!< subscriber of manually inserted waypoints
    ros::Subscriber external_path_sub_;  //!< subscriber of external input path
    ros::Publisher waypoint_marker_pub_;  //!< publisher of waypoint visualization markers
    ros::Publisher goal_pub_;  //!< publisher of goal corresponding to the final waypoint
    ros::Publisher plan_pub_;  //!< publisher of the global plan

    // configuration parameters
    double epsilon_;  //!< distance threshold between two waypoints that signifies the last waypoint
    int waypoints_per_meter_;  //!< number of waypoints per meter of generated path used for interpolation

    // containers
    std::vector<geometry_msgs::PoseStamped> waypoints_;  //!< container for the manually inserted waypoints
    nav_msgs::Path path_;  //!< container for the generated interpolated path

    //flags
    bool clear_waypoints_;  //!< flag indicating that the waypoint container must be cleared to start anew
};

}  // namespace waypoint_global_planner

#endif  // WAYPOINT_GLOBAL_PLANNER_H

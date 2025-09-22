#include "planner_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath> 

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

// Dummy planPath function 
nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map, 
    const geometry_msgs::msg::Pose& start, 
    const geometry_msgs::msg::PointStamped& goal
)
{
        nav_msgs::msg::Path path;
    path.header.frame_id = map.header.frame_id;
    path.header.stamp = rclcpp::Clock().now();

    // Add start pose
    geometry_msgs::msg::PoseStamped start_pose;
    start_pose.pose = start;
    path.poses.push_back(start_pose);

    // Add goal pose
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.pose.position = goal.point;
    path.poses.push_back(goal_pose);
    goal_pose.header = path.header;
    start_pose.header = path.header;


    RCLCPP_INFO(logger_, "Dummy path created from start to goal.");

    return path;
}

} 

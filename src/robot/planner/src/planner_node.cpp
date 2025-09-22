#include "planner_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>

PlannerNode::PlannerNode() 
: Node("planner"), planner_(robot::PlannerCore(this->get_logger())), state_(State::WAITING_FOR_GOAL)
{
    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

    // Publisher
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

// Map callback 
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    current_map_ = *msg; 

    // If we are moving towards a goal, replan 
    if (goal_received_)
        publishPath(); 
}

// Goal callback
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    goal_ = *msg;
    goal_received_ = true;
    
    // Transition to FOLLOWING_PATH state
    state_ = State::FOLLOWING_PATH;

    RCLCPP_INFO(this->get_logger(), "Received new goal");
    publishPath();
}

// Odometry callback 
void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_pose_ = msg->pose.pose; 
}

// Timer callback 
void PlannerNode::timerCallback()
{
    if(state_ == State::FOLLOWING_PATH)
    {
        if(goalReached())
        {
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            goal_received_ = false;
            state_ = State::WAITING_FOR_GOAL;  // back to waiting
        }
        else
        {
            // Optional: replan if robot is stuck or map updated
            publishPath();
        }
    }
}


// Check if goal is reached
bool PlannerNode::goalReached()
{
    double dx = goal_.point.x - robot_pose_.position.x; 
    double dy = goal_.point.y - robot_pose_.position.y; 
    return std::sqrt(dx*dx + dy*dy) < 0.5; // threshold
}

// Call planner and publish path
void PlannerNode::publishPath() 
{
    if (!goal_received_ || current_map_.data.empty())
        return; 

    nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_);
    path_pub_->publish(path); 
}

// Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}

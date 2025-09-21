#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    // Place callback function here
    void publishCostMap();
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    int width = 100;
    int height = 100;
    float resolution = 0.1f;
    float inflation_radius = 1.0f;
    std::vector<std::vector<int>> OccupancyMap;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif 
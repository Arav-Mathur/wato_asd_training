#include <cmath>
#include <chrono>

#include "map_memory_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {

    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/costmap", 10,std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom/filtered", 10,std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&MapMemoryNode::updateMap, this));
    global_map_.info.resolution = 0.1; 
    global_map_.info.width = 100;     
    global_map_.info.height = 100;    
    global_map_.info.origin.position.x = -5;
    global_map_.info.origin.position.y = -5;
    global_map_.info.origin.position.z = 0.0;
    global_map_.info.origin.orientation.x = 0.0;
    global_map_.info.origin.orientation.y = 0.0;
    global_map_.info.origin.orientation.z = 0.0;
    global_map_.info.origin.orientation.w = 1.0;
    global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1);
  }

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  latest_costmap_ = *msg;
  costmap_updated_ = true;
}
void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    double distance = std::sqrt(std::pow(x - last_x_, 2) + std::pow(y - last_y_, 2));

    if (distance >= distance_threshold_) {
        last_x_ = x;
        last_y_ = y;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
  if (should_update_map_ && costmap_updated_) {

    for (int y = 0; y < static_cast<int>(latest_costmap_.info.height); y++) {
      for (int x = 0; x < static_cast<int>(latest_costmap_.info.width); x++) {
        int idx = y * latest_costmap_.info.width + x;
        int8_t cell = latest_costmap_.data[idx];        

        double local_map_x = x * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.x;
        double local_map_y = y * latest_costmap_.info.resolution + latest_costmap_.info.origin.position.y;

        double global_x = last_x_ + local_map_x;
        double global_y = last_y_ + local_map_y;

        int g_x_idx = static_cast<int>((global_x - global_map_.info.origin.position.x) / global_map_.info.resolution);
        int g_y_idx = static_cast<int>((global_y - global_map_.info.origin.position.y) / global_map_.info.resolution);

        if (g_x_idx < 0 || g_x_idx >= static_cast<int>(global_map_.info.width) ||g_y_idx < 0 || g_y_idx >= static_cast<int>(global_map_.info.height)) {continue;}

        int g_idx = g_y_idx * global_map_.info.width + g_x_idx;

        if (cell > global_map_.data[g_idx]) {
          global_map_.data[g_idx] = cell;
        }
      }
    }

    global_map_.header.stamp = this->now();
    global_map_.header.frame_id = "map";

    map_pub_->publish(global_map_);

    should_update_map_ = false;
    costmap_updated_ = false;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}



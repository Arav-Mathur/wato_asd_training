#include <chrono>
#include <memory>
#include <vector>
#include <cmath>

#include "costmap_node.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {

  OccupancyMap.resize(height, std::vector<int>(width, 0));
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostMap, this));
}
void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  for (auto &row : OccupancyMap) std::fill(row.begin(), row.end(), 0.0);

  for (size_t i = 0; i < scan->ranges.size(); i++)
  {
    float angle = scan->angle_min + (i * scan->angle_increment);
    float range = scan->ranges[i];
    if (range < scan->range_max && range > scan->range_min)
    {
      int x_grid =  static_cast<int>((range * cos(angle))/resolution);
      int y_grid =  static_cast<int>((range * sin(angle))/resolution);
      OccupancyMap[y_grid][x_grid] = 100;
     }
    
  }
  
  int inflation_cells = static_cast<int>(inflation_radius / resolution);
  for (size_t y = 0; y < height; y++){
    for (size_t x = 0; x < width; x++){
      if(OccupancyMap[y][x] == 100){
          for (size_t sy = -inflation_cells; sy <= inflation_cells; sy++){
            for (size_t sx = -inflation_cells; sx <= inflation_cells; sx++){
              if ( x+sx >= 0 && x+sx < width && y+sy >= 0 && y+sy < height ){ //check in map
                float e_distance = sqrt((sx*sx + sy*sy)) * resolution;
                if(e_distance<inflation_radius){
                  float cost = 100*(1-(e_distance/inflation_radius));
                  if (cost> OccupancyMap[y+sy][x+sx]){
                    OccupancyMap[y+sy][x+sx] = static_cast<int>(cost);
                  }
                }
              }
            }
          }
      }
    }
    
  }
}
// Define the timer to publish a message every 500ms
void CostmapNode::publishCostMap() {
  nav_msgs::msg::OccupancyGrid msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  msg.info.resolution = 0.1;
  msg.info.width = width;
  msg.info.height = height;

  msg.info.origin.position.x = 0.0;
  msg.info.origin.position.y = 0.0;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.data.resize(width * height);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            msg.data[y * width + x] = static_cast<int8_t>(OccupancyMap[y][x]);
        }
    }
  costmap_pub_->publish(msg);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
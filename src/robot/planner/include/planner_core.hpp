#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Function to compute path using A*
    nav_msgs::msg::Path planPath(
      const nav_msgs::msg::OccupancyGrid& map,
      const geometry_msgs::msg::Pose& start,
      const geometry_msgs::msg::PointStamped& goal
    );

  private:
    rclcpp::Logger logger_;
};

}

#endif

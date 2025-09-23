#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <vector>
#include <unordered_set>
#include <queue>
#include <functional>

namespace robot {

// Node structure for A* algorithm
struct Node {
    int x, y;             // grid coordinates
    double g;             // cost from start
    double h;             // heuristic cost to goal
    double f;             // total cost = g + h
    Node* parent;         // pointer to parent node

    Node(int xx, int yy, double gg = 0, double hh = 0, Node* p = nullptr)
        : x(xx), y(yy), g(gg), h(hh), f(gg + hh), parent(p) {}
};

// Comparison functor for priority queue (min-heap by f)
struct CompareF {
    bool operator()(Node* a, Node* b) {
        return a->f > b->f;
    }
};

// Cell index for hashing in closed list
struct CellIndex {
    int x, y;
    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    bool operator==(const CellIndex& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash function for CellIndex
struct CellIndexHash {
    std::size_t operator()(const CellIndex& ci) const {
        return std::hash<int>()(ci.x) ^ (std::hash<int>()(ci.y) << 1);
    }
};

class PlannerCore {
public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Plan a path from start to goal using A*
    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& map,
                                 const geometry_msgs::msg::Pose& start,
                                 const geometry_msgs::msg::PointStamped& goal);

    // Convert world coordinates to grid coordinates
    std::pair<int,int> worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid& map);

    // Convert grid coordinates to world coordinates
    std::pair<double,double> gridToWorld(int i, int j, const nav_msgs::msg::OccupancyGrid& map);

    // Heuristic function (Euclidean distance)
    double heuristic(int x1, int y1, int x2, int y2);

    // Get neighboring nodes for A*
    std::vector<Node*> getNeighbors(Node* current, 
                                   const nav_msgs::msg::OccupancyGrid& map,
                                   int goal_i, int goal_j);

private:
    rclcpp::Logger logger_;
};

} // namespace robot

#endif // PLANNER_CORE_HPP_

#include "planner_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <queue>
#include <vector>
#include <unordered_set>
#include <algorithm>

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
    : logger_(logger) {}


nav_msgs::msg::Path PlannerCore::planPath(
    const nav_msgs::msg::OccupancyGrid& map,
    const geometry_msgs::msg::Pose& start,
    const geometry_msgs::msg::PointStamped& goal)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = map.header.frame_id;
    path.header.stamp = rclcpp::Clock().now();

    // Convert start and goal from world to grid
    auto [start_i, start_j] = worldToGrid(start.position.x, start.position.y, map);
    auto [goal_i, goal_j]   = worldToGrid(goal.point.x, goal.point.y, map);

    std::priority_queue<Node*, std::vector<Node*>, CompareF> openList;
    std::unordered_set<CellIndex, CellIndexHash> closedList;

    Node* startNode = new Node(start_i, start_j, 0, heuristic(start_i, start_j, goal_i, goal_j));
    openList.push(startNode);

    Node* goalNode = nullptr;

    while (!openList.empty()) {
        Node* current = openList.top();
        openList.pop();

        CellIndex idx(current->x, current->y);
        if (closedList.find(idx) != closedList.end()) continue;
        closedList.insert(idx);

        if (current->x == goal_i && current->y == goal_j) {
            goalNode = current;
            break;
        }

        for (auto& neighbor : getNeighbors(current, map, goal_i, goal_j)) {
            openList.push(neighbor);
        }
    }

    if (goalNode) {
        Node* node = goalNode;
        while (node) {
            auto [x_world, y_world] = gridToWorld(node->x, node->y, map);
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = x_world;
            pose.pose.position.y = y_world;
            pose.header = path.header;
            path.poses.push_back(pose);
            node = node->parent;
        }
        std::reverse(path.poses.begin(), path.poses.end());
        RCLCPP_INFO(logger_, "Path found!");
    } else {
        RCLCPP_WARN(logger_, "No path found!");
    }

    // Cleanup nodes in openList
    while (!openList.empty()) {
        delete openList.top();
        openList.pop();
    }

    return path;
}

std::pair<int,int> PlannerCore::worldToGrid(double x, double y, const nav_msgs::msg::OccupancyGrid& map) {
    int i = static_cast<int>((y - map.info.origin.position.y) / map.info.resolution);
    int j = static_cast<int>((x - map.info.origin.position.x) / map.info.resolution);
    return {i, j};
}

std::pair<double,double> PlannerCore::gridToWorld(int i, int j, const nav_msgs::msg::OccupancyGrid& map) {
    double x = j * map.info.resolution + map.info.origin.position.x;
    double y = i * map.info.resolution + map.info.origin.position.y;
    return {x, y};
}

double PlannerCore::heuristic(int x1, int y1, int x2, int y2) {
    return std::sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

std::vector<Node*> PlannerCore::getNeighbors(Node* current, const nav_msgs::msg::OccupancyGrid& map, int goal_i, int goal_j) {
    std::vector<Node*> neighbors;
    std::vector<std::pair<int,int>> moves = {{0,1},{1,0},{0,-1},{-1,0}}; // 4-connected grid

    for (auto [dx, dy] : moves) {
        int nx = current->x + dx;
        int ny = current->y + dy;

        if (nx < 0 || nx >= map.info.width || ny < 0 || ny >= map.info.height) continue;

        int index = ny * map.info.width + nx;
        if (map.data[index] != 0) continue; // obstacle

        Node* neighbor = new Node(nx, ny, current->g + 1, heuristic(nx, ny, goal_i, goal_j), current);
        neighbors.push_back(neighbor);
    }

    return neighbors;
}

} // namespace robot

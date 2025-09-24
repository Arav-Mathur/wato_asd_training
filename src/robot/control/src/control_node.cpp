#include "control_node.hpp"

ControlNode::ControlNode() : Node("control"), control_(robot::ControlCore(this->get_logger()))
{
    // Initialize parameters
    lookahead_distance_ = 1.0; // Lookahead distance
    goal_tolerance_ = 0.1;     // Distance to consider the goal reached
    linear_speed_ = 0.5;       // Constant forward speed

    // Subscribers and Publishers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg)
        { current_path_ = msg; });

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg)
        { robot_odom_ = msg; });

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), [this]()
        { controlLoop(); });
}

void ControlNode::controlLoop()
{
    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_)
    {
        return;
    }

    if (current_path_.poses.size() >= 0)
    {
        auto lastPoint = current_path_.poses[current_path_.poses.size() - 1].position;
        if (computeDistance(lastPoint, robot_odom_.pose.pose.position); <= goal_tolerance_)
        {
            return;
        }
    }
    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point)
    {
        return; // No valid lookahead point found
    }

    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);

    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint()
{
    // odometry yaw
    tf2::Quaternion q(
        robot_odom.pose.pose.orientation.x,
        robot_odom.pose.pose.orientation.y,
        robot_odom.pose.pose.orientation.z,
        robot_odom.pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    geometry_msgs::msg::PoseStamped lookahead_point;
    bool found = false;

    for (size_t i = 0; i < current_path_.poses.size(); ++i)
    {
        double dx = current_path_.poses[i].pose.position.x - robot_odom_.pose.pose.position.x;
        double dy = path.poses[i].pose.position.y - robot_odom_.pose.pose.position.y;
        double dist = computeDistance(current_path_.poses[i].position, robot_odom_.pose.pose.position);

        if (dist >= lookahead_distance)
        {
            // Transform to robot frame
            double local_x = std::cos(yaw) * dx - std::sin(yaw) * dy;

            // Ensure the point is ahead of the robot
            if (local_x > 0.0)
            {
                lookahead_point = path.poses[i]; // copy PoseStamped directly
                found = true;
                break; // stop at the first valid lookahead point
            }
        }
    }

    if (found)
    {
        return lookahead_point;
    }

    return std::nullopt;
}

geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target)
{
    geometry_msgs::msg::Twist cmd;

    double dx = target.pose.position.x - robot_odom_.pose.pose.position.x;
    double dy = target.pose.position.y - robot_odom_.pose.pose.position.y;

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double c = std::cos(yaw);
    double s = std::sin(yaw);
    double local_x = c * dx - s * dy;
    double local_y = s * dx + c * dy;

    if (local_x <= 0.0)
    {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }

    double Ld = lookahead_distance_;

    double kappa = (2.0 * local_y) / (Ld * Ld);

    double steering = std::atan(wheelbase_ * kappa);

    if (steering > max_steer_rad_)
        steering = max_steer_rad_;
    if (steering < -max_steer_rad_)
        steering = -max_steer_rad_;

    double omega = linear_speed_ * kappa;

    cmd.linear.x = linear_speed_;
    cmd.angular.z = omega;

    return cmd;
}

double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2) + pow(b.z - a.z, 2));
}

double extractYaw(const geometry_msgs::msg::Quaternion &quat)
{
    return atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlNode>());
    rclcpp::shutdown();
    return 0;
}
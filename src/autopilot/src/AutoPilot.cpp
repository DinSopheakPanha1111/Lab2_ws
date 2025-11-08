#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

class AutoPilot : public rclcpp::Node
{
public:
    AutoPilot() : Node("autopilot"), last_turn_direction_(0), total_distance_(0.0), exploration_done_(false)
    {
        // Subscribers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&AutoPilot::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&AutoPilot::odom_callback, this, std::placeholders::_1));

        // Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "AutoPilot node started.");
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        if (!path_points_.empty())
        {
            auto [prev_x, prev_y] = path_points_.back();
            total_distance_ += std::hypot(x - prev_x, y - prev_y);
        }
        path_points_.push_back({x, y});

        // Stop condition — for example, total distance > 10 meters
        if (total_distance_ > 10.0 && !exploration_done_)
        {
            RCLCPP_WARN(this->get_logger(), "Exploration finished (area coverage threshold reached).");
            stop_robot();
            exploration_done_ = true;
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (exploration_done_)
            return; // Stop processing once done

        geometry_msgs::msg::Twist cmd_msg;

        // Define angle ranges (front ±20°)
        const float right_angle_min = -20.0 * M_PI / 180.0;
        const float right_angle_max = 0.0;
        const float left_angle_min  = 0.0;
        const float left_angle_max  = 20.0 * M_PI / 180.0;

        auto get_index = [&](float angle) -> int {
            return static_cast<int>((angle - msg->angle_min) / msg->angle_increment);
        };

        int right_start = std::max(0, get_index(right_angle_min));
        int right_end   = std::min(static_cast<int>(msg->ranges.size()) - 1, get_index(right_angle_max));
        int left_start  = std::max(0, get_index(left_angle_min));
        int left_end    = std::min(static_cast<int>(msg->ranges.size()) - 1, get_index(left_angle_max));

        float min_right = std::numeric_limits<float>::infinity();
        float min_left  = std::numeric_limits<float>::infinity();

        for (int i = right_start; i <= right_end; ++i)
            if (std::isfinite(msg->ranges[i])) min_right = std::min(min_right, msg->ranges[i]);
        for (int i = left_start; i <= left_end; ++i)
            if (std::isfinite(msg->ranges[i])) min_left = std::min(min_left, msg->ranges[i]);

        const float safe_distance = 0.3;
        const float hysteresis = 0.05;

        // Default: go forward slowly
        cmd_msg.linear.x = 0.05;
        cmd_msg.angular.z = 0.0;

        if (min_left < safe_distance && min_right < safe_distance)
        {
            cmd_msg.linear.x = 0.0;
            if (last_turn_direction_ == 0)
                last_turn_direction_ = 1; // default turn left
            cmd_msg.angular.z = (last_turn_direction_ == 1) ? 1.0 : -1.0;
        }
        else if (min_left < safe_distance && min_right > safe_distance + hysteresis)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = -2.0;
            last_turn_direction_ = -1;
        }
        else if (min_right < safe_distance && min_left > safe_distance + hysteresis)
        {
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 2.0;
            last_turn_direction_ = 1;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Left: %.2f | Right: %.2f | v=%.2f | w=%.2f | dist=%.2f m",
                    min_left, min_right, cmd_msg.linear.x, cmd_msg.angular.z, total_distance_);

        cmd_pub_->publish(cmd_msg);
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;
        cmd_pub_->publish(stop);
        RCLCPP_INFO(this->get_logger(), "Robot stopped.");
    }

    // Members
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    int last_turn_direction_;   // -1 = right, 1 = left
    double total_distance_;     // in meters
    bool exploration_done_;     // stop condition
    std::vector<std::pair<double, double>> path_points_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoPilot>());
    rclcpp::shutdown();
    return 0;
}


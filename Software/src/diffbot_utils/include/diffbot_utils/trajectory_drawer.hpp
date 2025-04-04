#ifndef TRAJECTORY_DRAWER_HPP
#define TRAJECTORY_DRAWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

class TrajectoryDrawer : public rclcpp::Node
{
    private:
        void odomCallback(const nav_msgs::msg::Odometry &msg);
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        nav_msgs::msg::Path trajectory_;

    public:
        TrajectoryDrawer();
};


#endif
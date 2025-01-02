#ifndef SIMPLE_CONTROLLER_HPP
#define SIMPLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>

class SimpleController : public rclcpp::Node{
    private:
        void velCallback(const geometry_msgs::msg::TwistStamped &msg);

    public:
        SimpleController(const std::string &name);
        
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

        double wheel_radius_;
        double wheel_separation_;
        Eigen::Matrix2d speed_conversion_;

};

#endif
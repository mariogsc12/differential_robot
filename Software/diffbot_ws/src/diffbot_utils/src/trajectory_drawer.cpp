#include "diffbot_utils/trajectory_drawer.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

TrajectoryDrawer::TrajectoryDrawer()
                : Node("trajectory_drawer")
{
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/diffbot_controller/odom",10,
                        std::bind(&TrajectoryDrawer::odomCallback,this,  _1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/diffbot_controller/trajectory",10);
}

void TrajectoryDrawer::odomCallback(const nav_msgs::msg::Odometry &msg)
{
    trajectory_.header.frame_id = msg.header.frame_id;      // same frame_id as odom msg

    geometry_msgs::msg::PoseStamped curr_pose;              
    curr_pose.header.frame_id = msg.header.frame_id;        // same frame_id as odom msg
    curr_pose.header.stamp = msg.header.stamp;              // same time as odom msg
    curr_pose.pose = msg.pose.pose;                         // same pose as odom msg
    trajectory_.poses.push_back(curr_pose);                 // add the current pose to the total trajectory

    path_pub_->publish(trajectory_);
}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryDrawer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
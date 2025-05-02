#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

void imuCallback(const sensor_msgs::msg::Imu & imu_msg)
{
    sensor_msgs::msg::Imu new_imu_msg;
    new_imu_msg = imu_msg;
    new_imu_msg.header.frame_id = "base_link_ekf";
    imu_pub_->publish(new_imu_msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc,argv);
    auto node = rclcpp::Node::make_shared("imu_republisher_node");
    rclcpp::sleep_for(1s);

    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu_ekf",10);
    auto imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>("imu_data",10, imuCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
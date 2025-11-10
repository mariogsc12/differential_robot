#ifndef MAPPING_WITH_KNOWN_POSES_HPP
#define MAPPING_WITH_KNOWN_POSES_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

namespace diffbot_mapping{

    struct Pose
    {
        Pose() = default;
        Pose(const int &px, const int &py) : x(px), y(py){};
        int x;
        int y;
    };

    unsigned int pose_to_cell(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info);

    Pose coordinates_to_pose(const double &px, const double &py, const nav_msgs::msg::MapMetaData &map_info);

    bool pose_on_map(const Pose &pose, const nav_msgs::msg::MapMetaData &map_info);

    std::vector<Pose> bresenham(const Pose &start, const Pose &end);

    std::vector<std::pair<Pose, double>> inverse_sensor_model(const Pose &p_robot, const Pose &p_beam);

    double prob2logodds(const double &p);

    double logodds2prob(const double &l);

    class MappingWithKnownPoses : public rclcpp::Node
    {
        public:
            MappingWithKnownPoses(const std::string &name);

        private:
            void scan_callback(const sensor_msgs::msg::LaserScan &scan);

            void timer_callback();

            nav_msgs::msg::OccupancyGrid map_;
            std::vector<double> probability_map_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
            rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
            rclcpp::TimerBase::SharedPtr timer_;
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    };
    
}

#endif // MAPPING_WITH_KNOWN_POSES_HPP
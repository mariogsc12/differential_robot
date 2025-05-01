#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Imu

imu_pub_ = None

def imuCallback(imu_msg):
    global imu_pub_
    imu_msg.header.frame_id = "base_link_ekf"
    imu_pub_.publish(imu_msg)

def main():
    global imu_pub_
    rclpy.init()
    node = Node("imu_republisher_node")
    time.sleep(1)

    node.declare_parameter("imu_topic", "imu/out")  # default = imu/out (simulation)
    imu_topic = node.get_parameter("imu_topic").get_parameter_value().string_value

    imu_pub_ = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub_ = node.create_subscription(Imu, imu_topic, imuCallback, 10)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
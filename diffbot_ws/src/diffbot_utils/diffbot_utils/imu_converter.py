#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf_transformations

class imuConverter(Node):
    def __init__(self):
        super().__init__("imu_converter")

        self.imu_sub_ = self.create_subscription(Imu, "imu_data", self.imuCallback, 10)
        self.imu_euler_pub_ = self.create_publisher(Vector3, "imu_euler", 10)

    def imuCallback(self, imu_data):
        q = imu_data.orientation
        quaternion = [q.x , q.y, q.z , q.w]

        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

        vector = Vector3(x=roll, y=pitch, z=yaw)

        self.imu_euler_pub_.publish(vector)


def main():
    rclpy.init()
    imu_converter = imuConverter()
    rclpy.spin(imu_converter)
    imu_converter.destroy_node()
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
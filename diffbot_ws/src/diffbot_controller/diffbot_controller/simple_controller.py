#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from tf_transformations import quaternion_from_euler


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_separation",0.26)

        self.wheel_radius = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value
        
        self.get_logger().info("Using wheel radius: %f" %self.wheel_radius)
        self.get_logger().info("Using wheel separation: %f" %self.wheel_separation)

        self.left_wheel_prev_pos_  = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.perv_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "diffbot_controller/odom", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "diffbot_controller/cmd_vel", self.velCallback,10)
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback,10)

        self.speed_conversion_ = np.array([[self.wheel_radius/2, self.wheel_radius/2],
                                          [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]])
        
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_link"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.get_logger().info("The conversion matrix is %s" %self.speed_conversion_)

    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_),robot_speed)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)


    def jointCallback(self, msg):
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.perv_time_

        self.left_wheel_prev_pos_ = msg.position[1]
        self.right_wheel_prev_pos_ = msg.position[0]
        self.perv_time_ = Time.from_msg(msg.header.stamp)

        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)

        linear = (self.wheel_radius * fi_right + self.wheel_radius * fi_left)/2
        angular = (self.wheel_radius * fi_right - self.wheel_radius * fi_left)/self.wheel_separation

        d_s = (self.wheel_radius*dp_right + self.wheel_radius*dp_left)/2
        d_theta = (self.wheel_radius*dp_right - self.wheel_radius*dp_left)/self.wheel_separation
        self.theta_+= d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0,0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.odom_pub_.publish(self.odom_msg_)




def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()
    

if __name__=="__main__":
    main()



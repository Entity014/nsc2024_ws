#!/usr/bin/env python3

import rclpy
import numpy as np
import transforms3d

from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool, Float64MultiArray
from rclpy import qos, Parameter


class CameraController(Node):
    def __init__(self):
        super().__init__("camera_controller_node")
        self.sub_camera_pose = self.create_subscription(
            Float32,
            "camera/pos",
            self.sub_camera_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_camera_pose
        self.pub_camera_pose = self.create_publisher(
            Float64MultiArray,
            "camera_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.pose = Float32()
        self.camera_pose = Float64MultiArray()

    def sub_camera_callback(self, data):
        print(data)
        self.pose = data

    def timer_callback(self):
        self.camera_pose.data = [np.deg2rad(self.pose.data)]
        self.pub_camera_pose.publish(self.camera_pose)


def main(args=None):
    rclpy.init(args=args)

    sub = CameraController()
    rclpy.spin(sub)
    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

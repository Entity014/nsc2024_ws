#!/usr/bin/env python3

import rclpy
import numpy as np
import transforms3d

from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool, Float64MultiArray
from rclpy import qos, Parameter


class ArmController(Node):
    def __init__(self):
        super().__init__("arm_controller_node")
        self.sub_imu1 = self.create_subscription(
            Imu,
            "imu1/filtered",
            self.sub_imu1_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_imu1
        self.sub_imu2 = self.create_subscription(
            Imu,
            "imu2/filtered",
            self.sub_imu2_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_imu2
        self.sub_encoder = self.create_subscription(
            Float32,
            "encoder/data",
            self.sub_encoder_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_encoder
        self.sub_limit = self.create_subscription(
            Bool,
            "limit/data",
            self.sub_limit_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_limit
        self.pub_left_arm = self.create_publisher(
            Float64MultiArray,
            "left_arm_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_right_arm = self.create_publisher(
            Float64MultiArray,
            "right_arm_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.limit = Bool()
        self.imu_arr1 = np.zeros((3))
        self.imu_arr2 = np.zeros((3))
        self.encoder = Float32()
        self.left_arm = Float64MultiArray()
        self.right_arm = Float64MultiArray()

    def timer_callback(self):
        print(self.imu_arr1)
        self.right_arm.data = [
            self.imu_arr1[2],
            self.imu_arr1[1],
            self.imu_arr1[0],
            np.deg2rad((self.encoder.data - 80) * 2),
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.pub_left_arm.publish(self.left_arm)
        self.pub_right_arm.publish(self.right_arm)

    def sub_imu1_callback(self, imu_data):
        orientation = [
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z,
            imu_data.orientation.w,
        ]
        euler = transforms3d.euler.quat2euler(orientation)
        self.imu_arr1[0] = euler[0]
        self.imu_arr1[1] = euler[1]
        self.imu_arr1[2] = euler[2]
        print(self.imu_arr1)

    def sub_imu2_callback(self, imu_data):
        orientation = [
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z,
            imu_data.orientation.w,
        ]
        euler = transforms3d.euler.quat2euler(orientation)
        self.imu_arr2[0] = euler[0]
        self.imu_arr2[1] = euler[1]
        self.imu_arr2[2] = euler[2]

    def sub_encoder_callback(self, encoder_data):
        self.encoder = encoder_data

    def sub_limit_callback(self, limit_data):
        self.limit = limit_data


def main(args=None):
    rclpy.init(args=args)

    sub = ArmController()
    rclpy.spin(sub)
    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

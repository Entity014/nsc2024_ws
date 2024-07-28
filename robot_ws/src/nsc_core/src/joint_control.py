#!/usr/bin/env python3

import rclpy
import numpy as np
import transforms3d

from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool, Float64MultiArray, String
from rclpy import qos, Parameter


class JointController(Node):
    def __init__(self):
        super().__init__("joint_controller_node")
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
        self.sub_state = self.create_subscription(
            String,
            "state/data",
            self.sub_state_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_state
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
        self.pub_left_leg = self.create_publisher(
            Float64MultiArray,
            "left_leg_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_right_leg = self.create_publisher(
            Float64MultiArray,
            "right_leg_controller/commands",
            qos_profile=qos.qos_profile_system_default,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.limit = Bool()
        self.imu_arr1 = np.zeros((3))
        self.imu_arr2 = np.zeros((3))
        self.encoder = Float32()
        self.state = String()
        self.left_arm = Float64MultiArray()
        self.left_leg = Float64MultiArray()
        self.right_arm = Float64MultiArray()
        self.right_leg = Float64MultiArray()
        self.left_hand = 0.0
        self.right_hand = 0.0

    def timer_callback(self):
        if self.state.data == "right_hand":
            self.right_arm.data = [
                0.0,
                self.imu_arr1[2],
                self.imu_arr1[0],
                self.encoder.data,
                0.0,
                self.imu_arr2[1],
                0.0,
                self.right_hand,
            ]
            self.right_leg.data = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        elif self.state.data == "right_leg":
            self.right_leg.data = [
                0.0,
                self.imu_arr1[2],
                self.imu_arr1[0],
                -self.encoder.data,
                0.0,
                0.0,
                0.0,
            ]
            self.right_arm.data = [
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        # print(self.imu_arr1)
        self.pub_left_arm.publish(self.left_arm)
        self.pub_left_leg.publish(self.left_leg)
        self.pub_right_arm.publish(self.right_arm)
        self.pub_right_leg.publish(self.right_leg)

    def sub_imu1_callback(self, imu_data):
        orientation = [
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z,
            imu_data.orientation.w,
        ]
        euler = transforms3d.euler.quat2euler(orientation)
        vector_x = [
            np.cos(euler[0]) * np.cos(euler[1]),
            np.sin(euler[0]) * np.cos(euler[1]),
            -np.sin(euler[1]),
        ]
        self.imu_arr1[0] = np.interp(
            np.arccos(vector_x[0]), [0, np.pi], [-np.pi / 2, np.pi / 2]
        )
        self.imu_arr1[1] = np.arccos(vector_x[2])
        self.imu_arr1[2] = constrain(-np.arccos(vector_x[2]), -np.pi, 0)

    def sub_imu2_callback(self, imu_data):
        orientation = [
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z,
            imu_data.orientation.w,
        ]
        euler = transforms3d.euler.quat2euler(orientation)
        self.imu_arr2[0] = euler[0]
        self.imu_arr2[1] = np.interp(euler[1], [-np.pi, np.pi], [-np.pi, np.pi])
        self.imu_arr2[2] = euler[2]

    def sub_encoder_callback(self, encoder_data):
        self.encoder = encoder_data
        self.encoder.data = float(
            constrain(np.deg2rad((self.encoder.data - 80) * 2), 0, np.pi)
        )

    def sub_limit_callback(self, limit_data):
        self.limit = limit_data
        if not self.limit.data:
            self.right_hand = 1.57
        else:
            self.right_hand = 0.0

    def sub_state_callback(self, state_data):
        self.state = state_data


def constrain(val, min_val, max_val):
    if val < min_val:
        return min_val
    elif val > max_val:
        return max_val
    else:
        return val


def main(args=None):
    rclpy.init(args=args)

    sub = JointController()
    rclpy.spin(sub)
    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

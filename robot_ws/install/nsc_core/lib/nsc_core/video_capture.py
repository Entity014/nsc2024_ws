#!/usr/bin/env python3
import rclpy
import os
import cv2
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy import qos


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.sub_cam = self.create_subscription(
            Image,
            "camera/image_raw",
            self.sub_cam_callback,
            10,
        )
        self.sub_cam
        self.sub_video = self.create_subscription(
            Bool,
            "camera/video",
            self.sub_video_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_video
        self.sub_image = self.create_subscription(
            Bool,
            "camera/image",
            self.sub_image_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_image
        self.bridge = CvBridge()
        self.video = Bool()
        self.image = Bool()
        self.video_writer = None
        self.state = 0
        self.save_time = ""
        self.recording = False

    def sub_video_callback(self, data):
        self.video = data

    def sub_image_callback(self, data):
        self.image = data

    def sub_cam_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        ros_time = self.get_clock().now().to_msg()
        timestamp = f"{ros_time.sec}_{ros_time.nanosec}"
        if self.image.data:
            save_path = os.path.expanduser("~/robot_ws/image")
            if not os.path.exists(save_path):
                os.makedirs(save_path)
            image_filename = os.path.join(save_path, f"{timestamp}.jpg")
            cv2.imwrite(image_filename, cv_image)

        if self.video.data:
            if not self.recording:
                if self.state == 0:
                    save_path = os.path.expanduser("~/robot_ws/video")
                    if not os.path.exists(save_path):
                        os.makedirs(save_path)
                    video_filename = os.path.join(save_path, f"{timestamp}")
                    self.out = cv2.VideoWriter(
                        f"{video_filename}.mp4",
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        30.0,
                        (640, 480),
                    )
                    self.state = 1
                self.recording = True

        elif not self.video.data:
            if self.recording:
                self.state = 0
                self.recording = False
                self.out.release()

        if self.recording:
            self.out.write(cv_image)

        cv2.imshow("Camera", cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    sub = CameraSubscriber()
    rclpy.spin(sub)
    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

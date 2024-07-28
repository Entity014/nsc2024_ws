import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoPublisher(Node):
    def __init__(self):
        super().__init__("video_publisher")
        self.publisher_ = self.create_publisher(Image, "video_frames", 10)
        self.br = CvBridge()
        self.cap = cv2.VideoCapture(0)
        self.recording = False
        self.out = None

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open video device.")
            exit()

        self.frame_width = int(self.cap.get(3))
        self.frame_height = int(self.cap.get(4))
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().error(
                "Error: Can't receive frame (stream end?). Exiting ..."
            )
            return

        # Publish the frame
        self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))

        # Display the resulting frame
        cv2.imshow("frame", frame)

        # Check for key press
        key = cv2.waitKey(1) & 0xFF

        # Start recording when 'r' is pressed
        if key == ord("r"):
            if not self.recording:
                self.out = cv2.VideoWriter(
                    "output.avi",
                    cv2.VideoWriter_fourcc(*"XVID"),
                    20.0,
                    (self.frame_width, self.frame_height),
                )
                self.recording = True
                self.get_logger().info("Recording started.")
            else:
                self.get_logger().info("Already recording.")

        # Stop recording when 's' is pressed
        if key == ord("s"):
            if self.recording:
                self.recording = False
                self.out.release()
                self.get_logger().info("Recording stopped.")
            else:
                self.get_logger().info("Not recording.")

        # Write the frame if recording
        if self.recording:
            self.out.write(frame)

        # Exit the loop when 'q' is pressed
        if key == ord("q"):
            self.cap.release()
            if self.out is not None:
                self.out.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

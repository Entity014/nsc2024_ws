#!/usr/bin/env python3

import sys
import rclpy
import numpy as np
import transforms3d
import time
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QPushButton,
    QStackedWidget,
    QLabel,
    QStatusBar,
    QHBoxLayout,
    QFileDialog,
    QSlider,
)
from PyQt5.QtGui import QFont, QFontMetrics
from PyQt5.QtCore import Qt, QUrl, QTimer
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent
from PyQt5.QtMultimediaWidgets import QVideoWidget


from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Bool, String
from rclpy import qos, Parameter


class CustomButton(QPushButton):
    def __init__(self, text, color, hover_color, pressed_color, font_size, parent=None):
        super().__init__(text, parent)
        self.setFont(QFont("Arial", font_size, QFont.Bold))
        font_metrics = QFontMetrics(self.font())
        text_width = font_metrics.boundingRect(text).width()
        text_height = font_metrics.boundingRect(text).height()
        padding = 20
        button_width = text_width + padding
        button_height = text_height + padding
        self.setStyleSheet(
            f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border-radius: 12px;
                border: 2px solid #555;
                font-family: Arial;
                font-size: {font_size}px;
                text-align: center;
                padding: {padding // 2}px;
            }}
            QPushButton:hover {{
                background-color: {hover_color};
                border: 2px solid #000;
            }}
            QPushButton:pressed {{
                background-color: {pressed_color};
                border: 2px solid #000;
            }}
        """
        )
        self.setMinimumSize(button_width, button_height)
        self.setMaximumSize(button_width, button_height)


class Recording(QMainWindow):
    def __init__(self, parent, window_type, node):
        super().__init__()
        self.parent = parent
        self.window_type = window_type
        self.node = node

        self.setGeometry(150, 150, 400, 300)
        self.setWindowTitle(f"Recording")

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        # Create buttons
        self.record_button = CustomButton(
            "Record", "green", "darkgreen", "lightgreen", 14, self
        )
        self.record_button.clicked.connect(self.toggle_record)

        self.stop_button = CustomButton(
            "Stop", "red", "darkred", "lightcoral", 14, self
        )
        self.stop_button.clicked.connect(self.stop_recording)

        self.screenshot_button = CustomButton(
            "Screenshot", "blue", "darkblue", "lightblue", 14, self
        )
        self.screenshot_button.clicked.connect(self.screenshot)

        self.return_button = CustomButton(
            "Return to Main Window", "black", "gray", "lightgray", 14, self
        )
        self.return_button.clicked.connect(self.return_to_main_window)

        # Horizontal layout for record and stop buttons
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.record_button)
        button_layout.addWidget(self.stop_button)

        # Right hand and right leg buttons
        self.right_hand_button = CustomButton(
            "Right Hand", "black", "gray", "lightgray", 14, self
        )
        self.right_leg_button = CustomButton(
            "Right Leg", "black", "gray", "lightgray", 14, self
        )
        self.right_hand_button.clicked.connect(self.right_hand_clicked)
        self.right_leg_button.clicked.connect(self.right_leg_clicked)

        # Horizontal layout for right hand and right leg buttons
        right_buttons_layout = QHBoxLayout()
        right_buttons_layout.addWidget(self.right_hand_button)
        right_buttons_layout.addWidget(self.right_leg_button)

        # Status labels
        self.record_screenshot_status = QLabel("Click Record or Stop")
        self.record_screenshot_status.setAlignment(Qt.AlignCenter)

        self.screenshot_status = QLabel("Click Screenshot to capture")
        self.screenshot_status.setAlignment(Qt.AlignCenter)

        self.right_hand_leg_status = QLabel("Select Right Hand or Right Leg")
        self.right_hand_leg_status.setAlignment(Qt.AlignCenter)

        # Slider for adjusting angle
        self.angle_slider = QSlider(Qt.Horizontal)
        self.angle_slider.setRange(-180, 180)
        self.angle_slider.setValue(0)
        self.angle_slider.valueChanged.connect(self.update_angle_status)

        self.angle_status = QLabel("Angle: 0°")
        self.angle_status.setAlignment(Qt.AlignCenter)

        # Main layout
        layout = QVBoxLayout()
        layout.addLayout(button_layout)
        layout.addWidget(self.record_screenshot_status, alignment=Qt.AlignCenter)
        layout.addWidget(self.screenshot_button, alignment=Qt.AlignCenter)
        layout.addWidget(self.screenshot_status, alignment=Qt.AlignCenter)
        layout.addWidget(self.angle_slider, alignment=Qt.AlignCenter)
        layout.addWidget(self.angle_status, alignment=Qt.AlignCenter)
        layout.addLayout(right_buttons_layout)
        layout.addWidget(self.right_hand_leg_status, alignment=Qt.AlignCenter)
        layout.addWidget(self.return_button, alignment=Qt.AlignCenter)
        self.central_widget.setLayout(layout)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

    def toggle_record(self):
        self.record_screenshot_status.setText("Recording ...")
        self.node.update_video_cap(True)

    def stop_recording(self):
        self.record_screenshot_status.setText("Stopped.")
        self.node.update_video_cap(False)

    def screenshot(self):
        self.screenshot_status.setText("Screenshot success.")
        self.node.update_image_cap(True)

    def right_hand_clicked(self):
        self.right_hand_leg_status.setText("Right Hand clicked.")
        self.node.update_state("right_hand")

    def right_leg_clicked(self):
        self.right_hand_leg_status.setText("Right Leg clicked.")
        self.node.update_state("right_leg")

    def update_angle_status(self):
        angle = self.angle_slider.value()
        self.angle_status.setText(f"Angle: {angle}°")
        self.node.update_cam_pos(angle)

    def return_to_main_window(self):
        self.parent.return_to_main_window()
        self.close()


class ResultsWindow(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.parent = parent
        self.setGeometry(150, 150, 600, 400)
        self.setWindowTitle("Results")

        self.layout = QVBoxLayout()

        # Path to save video
        self.save_path = ""

        # Open 3D Model File button
        self.open_3d_model_button = CustomButton(
            "Open 3D Model File", "purple", "darkpurple", "lightpurple", 14, self
        )
        self.open_3d_model_button.clicked.connect(self.open_3d_model_file)
        self.layout.addWidget(self.open_3d_model_button, alignment=Qt.AlignCenter)

        # Select Path to Save Video button
        self.select_path_button = CustomButton(
            "Select Path to Save Video", "orange", "darkorange", "lightorange", 14, self
        )
        self.select_path_button.clicked.connect(self.select_save_path)
        self.layout.addWidget(self.select_path_button, alignment=Qt.AlignCenter)

        # Open Video Folder button
        self.open_video_folder_button = CustomButton(
            "Open Video Folder", "green", "darkgreen", "lightgreen", 14, self
        )
        self.open_video_folder_button.clicked.connect(self.open_video_folder)
        self.open_video_folder_button.setEnabled(False)  # Initially disabled
        self.layout.addWidget(self.open_video_folder_button, alignment=Qt.AlignCenter)

        # Video player
        self.video_widget = QVideoWidget()
        self.layout.addWidget(self.video_widget)

        # Media player
        self.media_player = QMediaPlayer()
        self.media_player.setVideoOutput(self.video_widget)

        # Return button
        self.return_button = CustomButton(
            "Return to Main Window", "black", "gray", "lightgray", 14, self
        )
        self.return_button.clicked.connect(self.return_to_main_window)
        self.layout.addWidget(self.return_button, alignment=Qt.AlignCenter)

        self.setLayout(self.layout)

    def open_3d_model_file(self):
        file_name, _ = QFileDialog.getOpenFileName(
            self, "Open 3D Model File", "", "3D Model Files (*.obj *.stl)"
        )
        if file_name:
            print(f"Selected 3D model file: {file_name}")

    def select_save_path(self):
        self.save_path = QFileDialog.getExistingDirectory(
            self, "Select Path to Save Video"
        )
        if self.save_path:
            self.open_video_folder_button.setEnabled(
                True
            )  # Enable button if path is selected
            print(f"Save path selected: {self.save_path}")
        else:
            self.open_video_folder_button.setEnabled(
                False
            )  # Disable button if no path is selected

    def open_video_folder(self):
        if self.save_path:
            print(f"Opening video folder: {self.save_path}")
        else:
            print("No path selected to open video folder")

    def return_to_main_window(self):
        self.parent.return_to_main_window()
        self.close()


class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setGeometry(100, 100, 400, 300)  # Set geometry for consistency
        self.setWindowTitle("Main Window")

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.main_menu = QWidget()
        self.stack.addWidget(self.main_menu)
        self.create_main_menu()

        self.results_window = ResultsWindow(self)
        self.stack.addWidget(self.results_window)

        self.secondary1_window = Recording(self, 1, self.node)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(50)

    def create_main_menu(self):
        layout = QVBoxLayout()
        self.to_recording_button = CustomButton(
            "Recording", "black", "green", "lightgray", 30, self
        )
        self.to_recording_button.clicked.connect(self.recording_Window)

        self.to_results_button = CustomButton(
            "Result", "black", "blue", "lightgray", 30, self
        )
        self.to_results_button.clicked.connect(self.show_results_window)

        self.exit_button = CustomButton("Exit", "red", "gray", "black", 30, self)
        self.exit_button.clicked.connect(self.close)

        layout.addWidget(self.to_recording_button, alignment=Qt.AlignCenter)
        layout.addWidget(self.to_results_button, alignment=Qt.AlignCenter)
        layout.addWidget(self.exit_button, alignment=Qt.AlignCenter)
        self.main_menu.setLayout(layout)

    def show_results_window(self):
        self.stack.setCurrentWidget(self.results_window)

    def recording_Window(self):
        self.secondary1_pos = self.geometry().topLeft()
        self.secondary1_window.setGeometry(
            self.secondary1_pos.x(), self.secondary1_pos.y(), 400, 300
        )
        self.secondary1_window.show()
        self.hide()

    def return_to_main_window(self):
        self.show()
        self.stack.setCurrentWidget(self.main_menu)

    def update(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)


class GuiBioLink(Node):
    def __init__(self):
        super().__init__("bio_link_node")
        self.pub_camera_pos = self.create_publisher(
            Float32,
            "camera/pos",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_video_capture = self.create_publisher(
            Bool,
            "camera/video",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_image_capture = self.create_publisher(
            Bool,
            "camera/image",
            qos_profile=qos.qos_profile_system_default,
        )
        self.pub_state = self.create_publisher(
            String,
            "state/data",
            qos_profile=qos.qos_profile_system_default,
        )
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.video_cap = Bool()
        self.image_cap = Bool()
        self.state = String()
        self.cam_pos = Float32()

    def timer_callback(self):
        self.pub_camera_pos.publish(self.cam_pos)
        self.pub_video_capture.publish(self.video_cap)
        self.pub_camera_pos.publish(self.cam_pos)
        self.pub_state.publish(self.state)
        self.pub_image_capture.publish(self.image_cap)
        time.sleep(0.1)
        self.image_cap.data = False
        self.pub_image_capture.publish(self.image_cap)

    def update_cam_pos(self, value):
        self.cam_pos.data = float(value)

    def update_video_cap(self, value):
        self.video_cap.data = value

    def update_image_cap(self, value):
        self.image_cap.data = value

    def update_state(self, value):
        self.state.data = value


def main(args=None):
    rclpy.init(args=args)

    sub = GuiBioLink()

    app = QApplication(sys.argv)
    window = MainWindow(sub)
    window.show()
    sys.exit(app.exec_())

    sub.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton


class MyWindow(QWidget):
    def __init__(self):
        super().__init__()

        # Set up the main layout
        self.layout = QVBoxLayout()

        # Create the Record button
        self.record_button = QPushButton("Record")
        self.record_button.setStyleSheet("background-color: green; color: white;")
        self.record_button.clicked.connect(self.toggle_record)

        # Create the Screenshot button
        self.screenshot_button = QPushButton("Screenshot")
        self.screenshot_button.setStyleSheet("background-color: blue; color: white;")
        self.screenshot_button.clicked.connect(self.screenshot)

        # Add buttons to the layout
        self.layout.addWidget(self.record_button)
        self.layout.addWidget(self.screenshot_button)

        # Set the layout to the window
        self.setLayout(self.layout)

        # Set window properties
        self.setWindowTitle("BioLink")
        self.setGeometry(100, 100, 300, 200)

        # Track the recording state
        self.is_recording = False

    def toggle_record(self):
        if self.is_recording:
            self.record_button.setText("Record")
            self.record_button.setStyleSheet("background-color: green; color: white;")
            print("Stopped recording")
        else:
            self.record_button.setText("Stop Record")
            self.record_button.setStyleSheet("background-color: red; color: white;")
            print("Started recording")
        self.is_recording = not self.is_recording

    def screenshot(self):
        print("Screenshot button clicked")


# Main function to run the application
def main():
    app = QApplication(sys.argv)
    window = MyWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()

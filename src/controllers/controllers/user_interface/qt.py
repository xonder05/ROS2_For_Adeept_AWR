from PyQt5.QtWidgets import QMainWindow, QLabel, QPushButton
from PyQt5 import uic
from PyQt5.QtCore import QByteArray
import base64
from PyQt5.QtGui import QPixmap, QImage, QImageReader

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        uic.loadUi('/home/daniel/ros2_ws/src/controllers/controllers/user_interface/ui.ui', self)

        self.gamepad_start_button.clicked.connect(lambda: self.node.gamepad_toggle(True))
        self.gamepad_stop_button.clicked.connect(lambda: self.node.gamepad_toggle(False))

        self.line_following_start_button.clicked.connect(lambda: self.node.line_following_toggle(True))
        self.line_following_stop_button.clicked.connect(lambda: self.node.line_following_toggle(False))

        self.wandering_start_button.clicked.connect(lambda: self.node.wandering_toggle(True))
        self.wandering_stop_button.clicked.connect(lambda: self.node.wandering_toggle(False))

        self.keyboard_start_button.clicked.connect(lambda: self.node.keyboard_toggle(True))
        self.keyboard_stop_button.clicked.connect(lambda: self.node.keyboard_toggle(False))

        self.video_stream_label.setPixmap(QPixmap("/home/daniel/ros2_ws/src/controllers/controllers/user_interface/camera_placeholder.png").scaled(int(round(self.size().width() / 2, 0)), int(round(self.size().height() / 2, 0))))
        self.video_stream_label.setScaledContents(True)

        self.show()

    def update_image(self, base64_image):
        base64_data = base64_image[len('data:image/jpeg;base64,'):]
        image_bytes = base64.b64decode(base64_data)
        pixmap = QPixmap()
        pixmap.loadFromData(image_bytes)
        pixmap = pixmap.scaled(int(round(self.size().width() / 2, 0)), int(round(self.size().height() / 2, 0)))
        self.video_stream_label.setPixmap(pixmap)
        self.video_stream_label.setGeometry(50, 150, pixmap.width() + 50, pixmap.height() + 150)  # Adjust the geometry as needed

    def update_ultrasonic_label(self, distance):
        self.ultrasonic_label.setText(str(distance))

    def update_line_tracking_label(self, left, middle, right):
        self.line_tracking_label.setText(str(left) + str(middle) + str(right))

    def update_wandering_state_label(self, state):
        self.wandering_state_label.setText(state)

    def update_line_tracking_state_label(self, state):
        self.line_following_state_label.setText(state)

    def put_node(self, node):
        self.node = node

    def closeEvent(self, event):
        self.node.destroy_node()
        event.accept()
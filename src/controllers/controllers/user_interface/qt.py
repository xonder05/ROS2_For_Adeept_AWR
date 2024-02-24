from PyQt5.QtWidgets import QMainWindow, QLabel, QPushButton
from PyQt5 import uic
from PyQt5.QtCore import QByteArray
from PyQt5.QtGui import QPixmap, QImage, QImageReader
from PyQt5.QtGui import QPixmap, QPainter, QColor, QPen, QPainterPath
from PyQt5.QtCore import Qt

import math

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

        self.line_tracking_left_label.setPixmap(self.create_vector_image())
        self.line_tracking_middle_label.setPixmap(self.create_vector_image())
        self.line_tracking_right_label.setPixmap(self.create_vector_image())

        self.ultrasonic_distance_indicator.setMaximum(450)
        self.ultrasonic_distance_indicator.setValue(0)
        self.ultrasonic_distance_indicator.setFormat("")

        self.show()

    def update_image(self, np_image_data_array):
        image = QImage.fromData(np_image_data_array)
        pixmap = QPixmap.fromImage(image)
        self.video_stream_label.setPixmap(pixmap)

    def update_ultrasonic_label(self, distance):
        if math.isinf(distance):
            self.ultrasonic_distance_indicator.setValue(450)
            self.ultrasonic_distance_indicator.setFormat(f"{distance}")
        else:
            self.ultrasonic_distance_indicator.setValue(int(distance * 100))
            self.ultrasonic_distance_indicator.setFormat(f"{round(distance, 2)}m")

    def update_line_tracking_label(self, left, middle, right):
        self.line_tracking_left_label.setPixmap(self.create_vector_image(left))
        self.line_tracking_middle_label.setPixmap(self.create_vector_image(middle))
        self.line_tracking_right_label.setPixmap(self.create_vector_image(right))
        
    def update_wandering_state_label(self, state):
        self.wandering_state_label.setText(state)

    def update_line_tracking_state_label(self, state):
        self.line_following_state_label.setText(state)

    def put_node(self, node):
        self.node = node

    def closeEvent(self, event):
        self.node.destroy_node()
        event.accept()

    def create_vector_image(self, filled=False):
        pixmap = QPixmap(int(self.size().width() / 25), int(self.size().width() / 25))
        pixmap.fill(Qt.transparent)

        pen = QPen(Qt.black)
        pen.setWidth(2)

        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(pen)

        if filled:
            painter.setBrush(QColor(Qt.black))
        else:
            painter.setBrush(Qt.NoBrush)

        painter.drawRect(pixmap.rect())

        painter.end()

        return pixmap
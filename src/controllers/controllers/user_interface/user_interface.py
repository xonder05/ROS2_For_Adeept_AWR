import sys
import threading

import rclpy
from controllers.user_interface.ui_bridge_node import UiBridgeNode

from PyQt5.QtWidgets import QApplication
from controllers.user_interface.qt import MainWindow

def main():
    rclpy.init()
    node = UiBridgeNode()

    app = QApplication(sys.argv)
    window = MainWindow()

    node.put_window(window)
    window.put_node(node)

    ros_thread_instance = threading.Thread(target=rclpy.spin, args=(node,))
    ros_thread_instance.start()
    
    app.exec_()

    rclpy.shutdown()
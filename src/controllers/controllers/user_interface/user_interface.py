import sys
import controllers.user_interface.global_variables

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

    while controllers.user_interface.global_variables.executeEventLoop:
        rclpy.spin_once(node, timeout_sec=0.001)
        app.processEvents()

    window.close()
    app.quit()
    node.destroy_node()
    rclpy.shutdown()
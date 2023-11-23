import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import DriveDirections

from flask import Flask, request, jsonify, Response, render_template

from adeept_awr_web_controller.camera import Camera

import multiprocessing

class FlaskWeb():
    def __init__(self, direction):
        self.direction = direction
        self.app = Flask(__name__)
        self.setup_routes()
        self.app.run(debug=True)

    def setup_routes(self):
        
        @self.app.route('/', methods=['GET'])
        def index():
            return render_template('index.html')

        @self.app.route('/direction-press', methods=['POST'])
        def direction_press():
            data = request.get_json()
            button_id = data['buttonId']
            action = data['action']

            if action == "press":
                self.direction.value = ord(button_id)
            else:
                self.direction.value = -1

            return ('', 200)
        
        def gen(camera):
            while True:
                frame = camera.get_actual_frame()
                yield (b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

        @self.app.route('/video_feed')
        def video_feed():
            return Response(gen(Camera()), mimetype='multipart/x-mixed-replace; boundary=frame')

class WebControllerNode(Node):
    def __init__(self):
        super().__init__("web_controller_node")
        self.publisher = self.create_publisher(DriveDirections, "drive_directions", 10)
        self.timer_ = self.create_timer(0.5, self.drive_direction_publisher)
        self.direction = multiprocessing.Value('i', -1)
        self.webServer = multiprocessing.Process(target=FlaskWeb, args=(self.direction,))
        self.webServer.start()

    def drive_direction_publisher(self):        
        if self.direction.value != -1:
            output = DriveDirections()
            output.direction = self.direction.value
            self.publisher.publish(output)

def main():
    rclpy.init()
    node = WebControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
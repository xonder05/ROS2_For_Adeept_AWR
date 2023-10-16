import rclpy
from rclpy.node import Node

from adeept_awr_interfaces.msg import DriveDirections

from flask import Flask, request, jsonify, Response, render_template

import subprocess
import os
import signal
import threading
import multiprocessing


data = [
    {"text": "Hello 1", "buttonText": "Button 1"},
    {"text": "word 2", "buttonText": "Button 2"},
    {"text": "yes 3", "buttonText": "Button 3"},
]

class FlaskWeb():
    def __init__(self, direction):
        self.direction = direction
        self.app = Flask(__name__)
        self.setup_routes()
        self.app.run()

    def setup_routes(self):

        @self.app.route('/direction-press', methods=['POST'])
        def direction_press():
            data = request.get_json()
            button_id = data['buttonId']
            action = data['action']

            if action == "press":
                print("button press")
                self.direction.value = ord(button_id)
                print(self.direction.value)
            else:
                print("button release")
                self.direction.value = -1
                print(self.direction.value)

            response_data = {'message': f"Received Button {button_id} {action} event."}
            return jsonify(response_data)


        @self.app.route('/get_data', methods=['GET'])
        def get_data():
            return jsonify(data)

        @self.app.route('/', methods=['GET'])
        def index():
            return render_template('index.html')


        @self.app.route('/run_command', methods=['GET'])
        def run_command():
            try:
                # Start the command in a subprocess, capturing its output
                process = subprocess.Popen("ls -la", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

                # Initialize variables to store output and errors
                output = ""
                errors = ""

                # Continuously read and capture the command's output
                while process.poll() is None:
                    line = process.stdout.readline()
                    output += line.decode('utf-8')  # Decode byte output to string
                    # You can send 'output' to your web interface for real-time updates

                # Capture any remaining output after the command finishes
                remaining_output, remaining_errors = process.communicate()
                output += remaining_output.decode('utf-8')
                errors = remaining_errors.decode('utf-8')

                return output, errors

            except subprocess.CalledProcessError as e:
                return None, str(e)
    
        @self.app.route('/execute', methods=['POST'])
        def execute_command():
            command = request.form['command']

            def execute_long_running_command():
                process = subprocess.Popen(
                    command,
                    shell=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True
                )

                for line in process.stdout:
                    yield line

            return Response(execute_long_running_command(), content_type='text/plain')


class WebControllerNode(Node):
    def __init__(self):
        super().__init__("web_controller_node")
        self.publisher = self.create_publisher(DriveDirections, "drive_directions", 10)
        self.timer_ = self.create_timer(0.5, self.drive_direction_publisher)
        self.direction = multiprocessing.Value('i', -1)
        #self.webServer = multiprocessing.Process(target=FlaskWeb(self.direction))
        self.webServer = multiprocessing.Process(target=FlaskWeb, args=(self.direction,))
        self.webServer.start()

    def drive_direction_publisher(self):
        print(self.direction.value)
        
        if self.direction.value != -1:
            output = DriveDirections()
            output.direction = self.direction.value
            self.publisher.publish(output)

def main():
    rclpy.init()
    node = WebControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
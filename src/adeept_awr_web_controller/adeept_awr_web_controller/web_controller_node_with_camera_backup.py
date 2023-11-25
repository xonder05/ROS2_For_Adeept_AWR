import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from adeept_awr_interfaces.msg import DriveDirections
from adeept_awr_interfaces.action import Servo

from flask import Flask, request, jsonify, Response, render_template

from adeept_awr_web_controller.camera import Camera

import multiprocessing

class FlaskWeb():
    def __init__(self, direction, servo_event, servo_queue):
        self.direction = direction
        self.servo_event = servo_event
        self.servo_queue = servo_queue
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
        
        @self.app.route('/servo-press', methods=['POST'])
        def servo_press():
            data = request.get_json()
            self.servo_queue.put(data['goal'])
            print(data['goal'])
            self.servo_event.set()
            self.servo_event.clear()


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
        self.timerSecond = self.create_timer(3, self.event_waiter)
        self.direction = multiprocessing.Value('i', -1)
        self.camera_angle_event = multiprocessing.Event()
        self.camera_angle_queue = multiprocessing.Queue()
        self.webServer = multiprocessing.Process(target=FlaskWeb, args=(self.direction, self.camera_angle_event, self.camera_angle_queue,))
        self.webServer.start()

    def event_waiter(self):
        while True:
            print("waiting")
            events = [self.camera_angle_event]
            triggered_event = multiprocessing.Event().wait(events, timeout=2)  # Wait with a timeout of 2 seconds

            if triggered_event is None:
                print("Receiver: Timeout occurred, no event received")
            else:
                print("servo event")
                if triggered_event == self.camera_angle_event:
                    data = self.camera_angle_queue.get()
                    self.send_goal(data)

                triggered_event.clear()


    def drive_direction_publisher(self):
        if self.direction.value != -1:
            output = DriveDirections()
            output.direction = self.direction.value
            self.publisher.publish(output)


    def send_goal(self, target_position):
        print(target_position)
        goal_msg = Servo.Goal()
        goal_msg.target_position = target_position

        self.action_client.wait_for_server()

        self.goal_future = self.action_client.send_goal_async(goal_msg, self.feedback_callback)

        self.goal_future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.curr_position))


def main():
    rclpy.init()
    node = WebControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
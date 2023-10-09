from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

class Service(Node):
    def __init__(self):
        super().__init__("service")
        self.srv = self.create_service(AddTwoInts, "adder", self.callback)


    def callback(self, request, response):
        response.sum = request.a + request.b
        return response
    
def main():
    rclpy.init()
    node = Service()
    rclpy.spin(node)
    rclpy.shutdown()


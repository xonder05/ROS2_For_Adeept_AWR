import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class Client(Node):
    def __init__(self):
        super().__init__("client")
        self.cli = self.create_client(AddTwoInts, "adder")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available waiting")
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main():
    rclpy.init()
    node = Client()
    response = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    node.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))    
    
    node.destroy_node()

    rclpy.shutdown()
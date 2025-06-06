import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Alternativ: dein eigener Service-Typ

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info(f'Received request: {request.a} + {request.b}')
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()

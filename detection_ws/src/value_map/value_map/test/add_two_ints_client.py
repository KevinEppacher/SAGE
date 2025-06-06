import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Warten auf Service-Verfügbarkeit
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.counter = 0
        self.timer = self.create_timer(3.0, self.call_service)

    def call_service(self):
        request = AddTwoInts.Request()
        request.a = self.counter
        request.b = self.counter + 1
        self.counter += 1

        self.get_logger().info(f'Sending request: {request.a} + {request.b}')
        future = self.client.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    rclpy.spin(client)
    rclpy.shutdown()

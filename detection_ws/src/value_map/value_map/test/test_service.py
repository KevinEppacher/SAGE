import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from seem_ros_interfaces.srv import Panoptic
from cv_bridge import CvBridge
import cv2


class PanopticClientNode(Node):
    def __init__(self):
        super().__init__('panoptic_client_node')
        self.get_logger().info('PanopticClientNode started.')

        # CV Bridge für Image-Konvertierung
        self.bridge = CvBridge()
        self.rgb_image = None

        # Subscriber für RGB
        self.subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_callback,
            10
        )

        # Service Client
        self.cli = self.create_client(Panoptic, 'panoptic_segmentation')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for panoptic_segmentation service...')

        # Timer mit 1 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        self.rgb_image = msg

    def timer_callback(self):
        if self.rgb_image is None:
            self.get_logger().warn('No image received yet.')
            return

        req = Panoptic.Request()
        req.image = self.rgb_image

        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            seg_image = self.bridge.imgmsg_to_cv2(response.panoptic_segmentation, desired_encoding='rgb8')
            cv2.imshow("Panoptic Segmentation", seg_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PanopticClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

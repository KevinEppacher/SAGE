import rclpy
from value_map.value_map import ValueMap

def main(args=None):
    rclpy.init(args=args)
    node = ValueMap()
    rclpy.spin(node)
    rclpy.shutdown()

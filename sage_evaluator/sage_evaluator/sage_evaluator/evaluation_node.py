#!/usr/bin/env python3
import rclpy
from sage_evaluator.evaluation import EvaluationDashboard

def main(args=None):
    rclpy.init(args=args)
    node = EvaluationDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction
import rclpy
from rclpy.node import Node as RclNode
from std_srvs.srv import Trigger


def call_semantic_map_service(_context, *args, **kwargs):
    """Immediately calls /openfusion_ros/run_semantic_map."""
    rclpy.init()
    node = RclNode("semantic_map_trigger_client")
    client = node.create_client(Trigger, "/openfusion_ros/run_semantic_map")

    node.get_logger().info("⏳ Waiting for /openfusion_ros/run_semantic_map service...")
    if not client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("❌ Service /openfusion_ros/run_semantic_map not available.")
        node.destroy_node()
        rclpy.shutdown()
        return

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"✅ Semantic mapping triggered: {future.result().message}")
    else:
        node.get_logger().error("❌ Failed to call semantic mapping trigger service.")

    node.destroy_node()
    rclpy.shutdown()


def generate_launch_description():
    """Launch description that only triggers the mapping service."""
    return LaunchDescription([
        OpaqueFunction(function=call_semantic_map_service)
    ])

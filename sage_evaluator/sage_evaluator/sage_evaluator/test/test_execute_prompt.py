#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sage_bt_msgs.action import ExecutePrompt
from sage_bt_msgs.srv import StartupCheck
import math
import os


class ExecutePromptTester(Node):
    def __init__(self):
        super().__init__("execute_prompt_tester")

        # --- Publishers and Clients ---
        self.path_pub = self.create_publisher(Path, "/test_bt_path", 10)
        self.bt_client = ActionClient(self, ExecutePrompt, "/sage_behaviour_tree/execute_prompt")
        self.startup_client = self.create_client(StartupCheck, "/sage_behaviour_tree/startup_check")

        # --- Configurable parameters ---
        self.prompt = "chair"
        self.experiment_id = "test_scene_001"
        self.save_directory = "/app/test.png"
        self.timeout = 10.0  # minutes

        # Trigger startup check after short delay
        self.timer = self.create_timer(1.0, self.run_once)
        self.once = False

    # ------------------------------------------------------------------
    def run_once(self):
        """Run startup check and send ExecutePrompt goal."""
        if self.once:
            return
        self.once = True

        # Wait for startup_check service
        if not self.startup_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("StartupCheck service not available.")
            return

        self.get_logger().info("Calling /sage_behaviour_tree/startup_check ...")
        req = StartupCheck.Request()
        future = self.startup_client.call_async(req)
        future.add_done_callback(self._on_startup_response)

    # ------------------------------------------------------------------
    def _on_startup_response(self, future):
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"StartupCheck call failed: {e}")
            return

        if not resp:
            self.get_logger().error("No response from StartupCheck.")
            return

        if resp.ready:
            self.get_logger().info("StartupCheck PASSED:")
        else:
            self.get_logger().warn("StartupCheck FAILED:")
        for line in resp.report.splitlines():
            self.get_logger().info(f"  {line}")

        if not resp.ready:
            self.get_logger().warn("Skipping ExecutePrompt due to failed StartupCheck.")
            return

        # Proceed with BT execution
        self._send_bt_goal()

    # ------------------------------------------------------------------
    def _send_bt_goal(self):
        """Send ExecutePrompt goal after StartupCheck."""
        if not self.bt_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecutePrompt action server not available.")
            return

        self.get_logger().info(f"Sending ExecutePrompt goal → '{self.prompt}'")

        goal = ExecutePrompt.Goal()
        goal.prompt = self.prompt
        goal.experiment_id = self.experiment_id
        goal.save_directory = self.save_directory
        goal.timeout = self.timeout

        send_future = self.bt_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    # ------------------------------------------------------------------
    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        # self.get_logger().info(f"[Feedback] active_node={fb.active_node}, status={fb.status}, log={fb.log}")

    # ------------------------------------------------------------------
    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the BT action server.")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    # ------------------------------------------------------------------
    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info("-----------------------------------------------------")
        self.get_logger().info("ExecutePrompt RESULT:")
        self.get_logger().info(f"  success          : {result.result}")
        self.get_logger().info(f"  confidence_score : {result.confidence_score:.3f}")
        self.get_logger().info(f"  total_time       : {result.total_time:.2f} s")
        self.get_logger().info(f"  path poses       : {len(result.accumulated_path.poses)}")
        self.get_logger().info(f"  start_pose (x,y) : ({result.start_pose.position.x:.2f}, {result.start_pose.position.y:.2f})")
        self.get_logger().info(f"  end_pose   (x,y) : ({result.end_pose.position.x:.2f}, {result.end_pose.position.y:.2f})")
        self.get_logger().info("-----------------------------------------------------")

        # Publish result path on /test_bt_path
        path = result.accumulated_path
        path.header.frame_id = "robot_original_pose_at_scan"
        path.header.stamp = self.get_clock().now().to_msg()

        for pose_stamped in path.poses:
            pose_stamped.header.frame_id = "robot_original_pose_at_scan"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

        self.path_pub.publish(path)
        self.get_logger().info(f"Published result path to /test_bt_path with {len(path.poses)} poses.")

        # Compute total path length
        total_len = self._compute_path_length(path)
        self.get_logger().info(f"Total path length: {total_len:.2f} m")

        # Clean shutdown
        self.get_logger().info("Shutting down test node.")
        self.destroy_node()
        self.create_timer(0.3, lambda: rclpy.shutdown())

    # ------------------------------------------------------------------
    def _compute_path_length(self, path: Path) -> float:
        if not path.poses:
            return 0.0
        dist = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            dist += math.sqrt(
                (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2
            )
        return dist


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ExecutePromptTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

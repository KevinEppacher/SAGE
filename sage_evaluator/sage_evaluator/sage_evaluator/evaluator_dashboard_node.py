#!/usr/bin/env python3
import os
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from evaluator_msgs.srv import GetShortestPath
from sage_bt_msgs.srv import StartupCheck
from sage_bt_msgs.action import ExecutePrompt
from nav_msgs.msg import Path
from sage_datasets.utils import DatasetManager
from multimodal_query_msgs.msg import SemanticPrompt

class Metrics:
    """Compute SR/SPL values and log via passed ROS2 node."""

    def __init__(self, node: Node, success: bool, actual_path: Path, shortest_path: Path):
        self.node = node
        self.success = success
        self.actual_path = actual_path
        self.shortest_path = shortest_path

        self.l_shortest = self._compute_length(shortest_path)
        self.l_actual = self._compute_length(actual_path)

        self.sr = float(success)
        self.spl = self._compute_spl()

        # Print immediately
        self._print_path_info()

    def _compute_length(self, path: Path) -> float:
        if path is None or not path.poses:
            return 0.0
        total = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            total += math.sqrt(
                (p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2
            )
        return total

    def _compute_spl(self) -> float:
        if not self.success:
            return 0.0

        if self.l_shortest <= 0.0 or self.l_actual <= 0.0:
            return 0.0

        return self.l_shortest / max(self.l_shortest, self.l_actual)

    def _print_path_info(self):
        actual_len = self.l_actual
        shortest_len = self.l_shortest

        self.node.get_logger().info("---- PATH METRICS ----")
        self.node.get_logger().info(f"Shortest path length: {shortest_len:.2f} m")
        self.node.get_logger().info(f"Actual executed path: {actual_len:.2f} m")
        self.node.get_logger().info(f"SR = {self.sr:.2f}, SPL = {self.spl:.3f}")
        self.node.get_logger().info("-----------------------")

    def to_dict(self):
        return {
            "SR": self.sr,
            "SPL": self.spl,
            "shortest_path_length": self.l_shortest,
            "actual_path_length": self.l_actual
        }



class EvaluatorDashboard(Node):
    """Central orchestrator node for evaluation experiments."""

    def __init__(self):
        super().__init__("evaluator_dashboard")

        # ---------------- Parameters ----------------
        self.declare_parameter("scene", "")
        self.declare_parameter("version", "")
        self.declare_parameter("episode_id", "")
        self.declare_parameter("prompt_set", "train")
        self.declare_parameter("debug_info", True)

        self.scene = self.get_parameter("scene").value
        self.version = self.get_parameter("version").value
        self.episode_id = self.get_parameter("episode_id").value
        self.prompt_set = self.get_parameter("prompt_set").value
        self.debug = self.get_parameter("debug_info").value

        # ---------------- Dataset loading ----------------
        self.dataset = DatasetManager(self.scene, self.version, self.episode_id)
        self.episode_dir = self.dataset.episode_dir()
        self.prompts_data = self.dataset.prompts()

        if "train" not in self.prompts_data or "eval" not in self.prompts_data:
            raise RuntimeError("prompts.json must contain both 'train' and 'eval' keys.")

        self.prompts_train = self.prompts_data["train"]
        self.prompts_eval = self.prompts_data["eval"]

        if len(self.prompts_train) != len(self.prompts_eval):
            raise RuntimeError("Number of train and eval prompts must match.")

        self.get_logger().info(f"Loaded {len(self.prompts_train)} paired prompts from dataset.")
        self.get_logger().info(f"Episode directory: {self.episode_dir}")

        # ---------------- Service and action clients ----------------
        self.shortest_client = self.create_client(GetShortestPath, "/evaluator/get_shortest_path")
        self.startup_check_client = self.create_client(StartupCheck, "/sage_behaviour_tree/startup_check")
        self.bt_action_client = ActionClient(self, ExecutePrompt, "/sage_behaviour_tree/execute_prompt")

        # Publisher for the actual executed path
        self.executed_path_pub = self.create_publisher(
            Path,
            "/evaluator/executed_path",
            10
        )

        # adding a zero shot publisher
        self.zero_shot_pub = self.create_publisher(
            SemanticPrompt,
            "/zero_shot_prompt",
            10
        )

        self._wait_for_servers()

        # ---------------- Metrics store ----------------
        self.metrics_all = {}

        # ---------------- Run full evaluation ----------------
        self.run_evaluation()

    # ------------------------------------------------------------
    def _wait_for_servers(self):
        self.get_logger().info("Waiting for service/action servers...")
        self.shortest_client.wait_for_service()
        self.startup_check_client.wait_for_service()
        self.bt_action_client.wait_for_server()
        self.get_logger().info("All service/action servers are ready.")

    # ------------------------------------------------------------
    def run_evaluation(self):
        total = len(self.prompts_train)

        for i, (train_prompt, eval_prompt) in enumerate(zip(self.prompts_train, self.prompts_eval), start=1):
            self.get_logger().info("=" * 60)
            self.get_logger().info(f"[{i}/{total}] EVALUATING: {train_prompt}")
            self.get_logger().info("=" * 60)
            zero_shot_msg = SemanticPrompt()
            zero_shot_msg.text_query = self.prompts_data["zero_shot"][i-1]
            self.zero_shot_pub.publish(zero_shot_msg)
            self.get_logger().info(f"Published zero-shot prompt: \"{zero_shot_msg.text_query}\" on topic /zero_shot_prompt")

            # --- 1. Get shortest path ---
            self.get_logger().info(f"Requesting shortest path for '{eval_prompt}'")
            shortest_path = self._call_shortest_path(eval_prompt)

            # --- 2. Startup check once before first BT run ---
            if i == 1:
                if not self._call_startup_check():
                    self.get_logger().error("Startup check failed, aborting evaluation.")
                    return

            # --- 3. Execute BT for prompt ---
            success, confidence, actual_path, start_pose, end_pose, total_time = self._call_execute_prompt(train_prompt)

            # --- 4. Compute metrics ---
            metrics = Metrics(self, success, actual_path, shortest_path)
            self.metrics_all[train_prompt] = metrics.to_dict()
            self.get_logger().info(f"Metrics for '{train_prompt}': {metrics.to_dict()}")

            # --- 5. Save per-prompt metrics JSON ---
            self._save_prompt_metrics(train_prompt, metrics)

        # --- 6. Save episode summary metrics ---
        self._save_summary_metrics()
        self.get_logger().info("All prompts completed. Evaluation finished.")

    # ------------------------------------------------------------
    def _call_shortest_path(self, query):
        req = GetShortestPath.Request()
        req.query = query
        future = self.shortest_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if not resp or not resp.success:
            self.get_logger().warn(f"Shortest path for '{query}' failed or timed out.")
            return Path()

        path_len = len(resp.path.poses)
        total_length = resp.path_length
        self.get_logger().info(f"Received path ({path_len} poses, {total_length:.2f} m)")
        return resp.path

    # ------------------------------------------------------------
    def _call_startup_check(self):
        req = StartupCheck.Request()
        self.get_logger().info("Performing BT startup check...")
        future = self.startup_check_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None:
            self.get_logger().error("No response from startup_check service.")
            return False

        ok = resp.ready
        self.get_logger().info(f"Startup check {'PASSED' if ok else 'FAILED'}: {resp.report}")
        return ok

    # ------------------------------------------------------------
    def _call_execute_prompt(self, query):
        goal = ExecutePrompt.Goal()
        goal.prompt = query
        goal.experiment_id = f"{self.scene}_{self.episode_id}"

        # Directory for detections
        det_dir = os.path.join(self.episode_dir, "detections", query)
        os.makedirs(det_dir, exist_ok=True)

        # Unique file per prompt
        filename = "detection.png"
        save_path = os.path.join(det_dir, filename)
        goal.save_directory = save_path
        goal.timeout = 30.0

        self.get_logger().info(f"Executing BT for prompt '{query}' → save {save_path}")
        send_future = self.bt_action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("BT goal rejected.")
            return False, 0.0, Path(), None, None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        success = result.result
        confidence = result.confidence_score    
        actual_path = result.accumulated_path
        start_pose = result.start_pose
        end_pose = result.end_pose
        total_time = result.total_time

        if actual_path is not None:
            # Ensure Header exists
            actual_path.header.stamp = self.get_clock().now().to_msg()
            if not actual_path.header.frame_id:
                actual_path.header.frame_id = "robot_original_pose_at_scan"  # or your global frame

            self.executed_path_pub.publish(actual_path)
            self.get_logger().info(
                f"Published executed path ({len(actual_path.poses)} poses) on /evaluator/executed_path"
            )

        self.get_logger().info(
            f"BT finished: result={success}, confidence={confidence:.3f}, poses={len(actual_path.poses)}, time={total_time:.2f}s"
        )

        self._save_detection_meta(query, success, "detection.png")

        return success, confidence, actual_path, start_pose, end_pose, total_time

    def _save_detection_meta(self, prompt, success, image_name):
        """Overwrite detection_meta.json with a single valid JSON object."""
        det_dir = os.path.join(self.episode_dir, "detections", prompt)
        os.makedirs(det_dir, exist_ok=True)

        meta_path = os.path.join(det_dir, "detection_meta.json")

        data = {
            "object": prompt,
            "status": "SUCCESS" if success else "FAILURE",
            "image": image_name
        }

        with open(meta_path, "w") as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"Saved detection meta → {meta_path}")

    # ------------------------------------------------------------
    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        # if self.debug:
        #     self.get_logger().info(
        #         f"[BT Feedback] active_node={fb.active_node}, status={fb.status}, log={fb.log}"
        #     )

    # ------------------------------------------------------------
    def _save_prompt_metrics(self, prompt, metrics: Metrics):
        """Save SR and SPL per prompt as JSON in its folder."""
        det_dir = os.path.join(self.episode_dir, "detections", prompt)
        os.makedirs(det_dir, exist_ok=True)
        with open(os.path.join(det_dir, "sr.json"), "w") as f:
            json.dump({"SR": metrics.sr}, f, indent=2)
        with open(os.path.join(det_dir, "spl.json"), "w") as f:
            json.dump({"SPL": metrics.spl}, f, indent=2)
        self.get_logger().info(f"Saved per-prompt metrics for '{prompt}' → {det_dir}")

    # ------------------------------------------------------------
    def _save_summary_metrics(self):
        """Save all metrics to a single JSON summary file in the episode root."""
        summary_path = os.path.join(self.episode_dir, "metrics.json")
        with open(summary_path, "w") as f:
            json.dump(self.metrics_all, f, indent=2)
        self.get_logger().info(f"Saved episode summary metrics → {summary_path}")


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorDashboard()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

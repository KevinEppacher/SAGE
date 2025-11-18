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


class Metrics:
    """Compute and hold SR/SPL values."""

    def __init__(self, success: bool, actual_path: Path, shortest_path: Path):
        self.success = success
        self.actual_path = actual_path
        self.shortest_path = shortest_path
        self.sr = float(success)
        self.spl = self._compute_spl()

    def _compute_length(self, path: Path) -> float:
        if path is None or not path.poses:
            return 0.0
        total = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i - 1].pose.position
            p2 = path.poses[i].pose.position
            total += math.sqrt(
                (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2
            )
        return total

    def _compute_spl(self) -> float:
        if not self.success:
            return 0.0
        l_shortest = self._compute_length(self.shortest_path)
        l_actual = self._compute_length(self.actual_path)
        if l_shortest <= 0.0 or l_actual <= 0.0:
            return 0.0
        return l_shortest / max(l_shortest, l_actual)

    def to_dict(self) -> dict:
        return {"SR": self.sr, "SPL": self.spl}


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
            metrics = Metrics(success, actual_path, shortest_path)
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
        existing = [f for f in os.listdir(det_dir) if f.endswith(".png")]
        filename = f"detection_{len(existing) + 1:04d}.png"
        save_path = os.path.join(det_dir, filename)
        goal.save_directory = save_path
        goal.timeout = 10.0

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

        self.get_logger().info(
            f"BT finished: result={success}, confidence={confidence:.3f}, poses={len(actual_path.poses)}, time={total_time:.2f}s"
        )

        return success, confidence, actual_path, start_pose, end_pose, total_time

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

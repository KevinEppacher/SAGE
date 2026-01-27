#!/usr/bin/env python3
import os
import json
import rclpy
import random
import tf2_ros

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from evaluator_msgs.srv import GetShortestPath
from sage_bt_msgs.srv import StartupCheck
from sage_bt_msgs.action import ExecutePrompt

from nav_msgs.msg import Path, OccupancyGrid
from sage_datasets.utils import DatasetManager
from sage_evaluator.utils.path_map_visualizer import PathMapVisualizer
from sage_evaluator.utils.metrics import Metrics
from sage_evaluator.utils.util import transform_path


class EvaluatorDashboard(Node):
    def __init__(self):
        super().__init__("evaluator_dashboard")

        # ---------------- Parameters ----------------
        self.declare_parameter("scene", "")
        self.declare_parameter("version", "")
        self.declare_parameter("episode_id", "")
        self.declare_parameter("debug_info", True)
        self.declare_parameter("prompt_shuffle_seed", 0)

        self.scene = self.get_parameter("scene").value
        self.version = self.get_parameter("version").value
        self.episode_id = self.get_parameter("episode_id").value
        self.debug = self.get_parameter("debug_info").value
        self.shuffle_seed = self.get_parameter("prompt_shuffle_seed").value

        # ---------------- Dataset loading ----------------
        self.dataset = DatasetManager(self.scene, self.version, self.episode_id)
        self.episode_dir = self.dataset.episode_dir()
        self.prompts_data = self.dataset.prompts()

        if not all(k in self.prompts_data for k in ["train", "eval", "zero_shot"]):
            raise RuntimeError("prompts.json must contain 'train', 'eval', and 'zero_shot'.")

        self.prompts_train = self.prompts_data["train"]
        self.prompts_eval = self.prompts_data["eval"]
        self.prompts_zero = self.prompts_data["zero_shot"]

        if not (len(self.prompts_train) == len(self.prompts_eval) == len(self.prompts_zero)):
            raise RuntimeError("train/eval/zero_shot prompt lists must have equal length.")

        # ---------------- Detection thresholds ----------------
        self.detection_threshold_initial = self.prompts_data.get(
            "detection_threshold_initial", 0.6
        )
        self.detection_threshold_final = self.prompts_data.get(
            "detection_threshold_final", 0.8
        )

        self.get_logger().info(
            f"Using detection thresholds: "
            f"initial={self.detection_threshold_initial:.2f}, "
            f"final={self.detection_threshold_final:.2f}"
        )

        self.get_logger().info(
            f"Loaded {len(self.prompts_train)} paired prompts from dataset."
        )
        self.get_logger().info(f"Episode directory: {self.episode_dir}")

        # ---------------- Service and action clients ----------------
        self.shortest_client = self.create_client(
            GetShortestPath, "/evaluator/get_shortest_path"
        )
        self.startup_check_client = self.create_client(
            StartupCheck, "/sage_behaviour_tree/startup_check"
        )
        self.bt_action_client = ActionClient(
            self, ExecutePrompt, "/sage_behaviour_tree/execute_prompt"
        )

        # Publisher for the executed path
        self.executed_path_pub = self.create_publisher(
            Path, "/evaluator/executed_path", 10
        )

        # Map + visualization
        self.map = None
        self.visualizer = PathMapVisualizer(self)

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            "/evaluator/map",
            self._map_callback,
            QoSProfile(
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
        )

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait for servers
        self._wait_for_servers()

        # Metrics store
        self.metrics_all = {}

        # Start evaluation
        self.run_evaluation()

    # ------------------------------------------------------------------
    # ADDITION: prompt ID helper (non-destructive feature addition)
    # ------------------------------------------------------------------
    def _make_prompt_id(self, index: int, prompt: str) -> str:
        """
        Create a zero-padded prompt identifier to avoid overwriting results.
        Example: 01_bed, 03_oven_and_stove
        """
        safe_prompt = prompt.replace(" ", "_")
        return f"{index:02d}_{safe_prompt}"

    # ------------------------------------------------------------------

    def _wait_for_servers(self):
        self.get_logger().info("Waiting for service/action servers...")
        self.shortest_client.wait_for_service()
        self.startup_check_client.wait_for_service()
        self.bt_action_client.wait_for_server()
        self.get_logger().info("All service/action servers are ready.")

    def run_evaluation(self):
        prompt_triplets = self._prepare_prompt_triplets()
        total = len(prompt_triplets)

        for i, (train_prompt, eval_prompt, zero_shot) in enumerate(
                prompt_triplets, start=1):

            # ADDITION: create unique prompt ID
            prompt_id = self._make_prompt_id(i, train_prompt)

            self.get_logger().info("=" * 60)
            self.get_logger().info(f"[{i}/{total}] EVALUATING: {prompt_id}")
            self.get_logger().info("=" * 60)

            # -------------------------------------------------
            # 1. Shortest path computation
            # -------------------------------------------------
            shortest_path = self._call_shortest_path(eval_prompt)
            if shortest_path is None:
                continue

            # -------------------------------------------------
            # 2. Startup check (only once)
            # -------------------------------------------------
            if i == 1:
                if not self._call_startup_check():
                    self.get_logger().error("Startup check failed, aborting evaluation.")
                    return

            # -------------------------------------------------
            # 3. Execute Behavior Tree
            # -------------------------------------------------
            result_data = self._call_execute_prompt(
                train_prompt,
                zero_shot,
                prompt_id
            )
            if result_data is None:
                continue

            # -------------------------------------------------
            # 4. Metrics computation
            # -------------------------------------------------
            metrics = Metrics(
                self,
                result_data["success"],
                result_data["path"],
                shortest_path
            )

            # ADDITION: use prompt_id as key (prevents overwrite)
            self.metrics_all[prompt_id] = {
                "prompt": train_prompt,
                **metrics.to_dict()
            }

            # -------------------------------------------------
            # 5. Path visualization (UNCHANGED behavior)
            # -------------------------------------------------
            plot_path = os.path.join(
                self.episode_dir,
                "detections",
                prompt_id,
                "paths.png"
            )

            actual_path_transformed = transform_path(
                self.tf_buffer,
                "robot_original_pose_at_scan",
                result_data["path"]
            )

            goal_pose = None
            if shortest_path and len(shortest_path.poses) > 0:
                goal_pose = shortest_path.poses[-1].pose

            self.visualizer.save_plot(
                plot_path,
                actual_path_transformed,
                shortest_path,
                actual_path_transformed.poses[0].pose,
                actual_path_transformed.poses[-1].pose,
                goal_pose
            )

            # -------------------------------------------------
            # 6. Save per-prompt metrics
            # -------------------------------------------------
            self._save_prompt_metrics(
                prompt_id,
                train_prompt,
                metrics,
                result_data
            )

        # -------------------------------------------------
        # 7. Save episode summary
        # -------------------------------------------------
        self._save_summary_metrics()
        self.get_logger().info("All prompts completed. Evaluation finished.")

    # ------------------------------------------------------------------

    def _call_shortest_path(self, query):
        req = GetShortestPath.Request()
        req.query = query
        future = self.shortest_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        if not resp or not resp.success or len(resp.path.poses) == 0:
            return None
        return resp.path

    def _call_startup_check(self):
        future = self.startup_check_client.call_async(StartupCheck.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        return resp is not None and resp.ready

    def _call_execute_prompt(self, query, zero_shot, prompt_id):
        goal = ExecutePrompt.Goal()
        goal.prompt = query
        goal.zero_shot_prompt = zero_shot
        goal.experiment_id = f"{self.scene}_{self.episode_id}"
        goal.timeout = 30.0

        goal.detection_threshold_initial = self.detection_threshold_initial
        goal.detection_threshold_final = self.detection_threshold_final

        det_dir = os.path.join(self.episode_dir, "detections", prompt_id)
        os.makedirs(det_dir, exist_ok=True)
        goal.save_directory = os.path.join(det_dir, "detection.png")

        send_future = self.bt_action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.accumulated_path and len(result.accumulated_path.poses) > 0:
            result.accumulated_path.header.stamp = self.get_clock().now().to_msg()
            result.accumulated_path.header.frame_id = "map"
            self.executed_path_pub.publish(result.accumulated_path)

        self._save_detection_meta(prompt_id, query, result.result, "detection.png")

        return {
            "success": result.result,
            "confidence": result.confidence_score,
            "detection_confidence": result.detection_confidence,
            "vlm_confidence": result.vlm_confidence,
            "memory_confidence": result.memory_confidence,
            "path": result.accumulated_path,
            "total_time": result.total_time
        }

    # ------------------------------------------------------------------

    def _save_detection_meta(self, prompt_id, prompt, success, image_name):
        det_dir = os.path.join(self.episode_dir, "detections", prompt_id)
        meta = {
            "prompt_id": prompt_id,
            "prompt": prompt,
            "status": "SUCCESS" if success else "FAILURE",
            "image": image_name
        }
        with open(os.path.join(det_dir, "detection_meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

    def _save_prompt_metrics(self, prompt_id, prompt, metrics, result_data):
        det_dir = os.path.join(self.episode_dir, "detections", prompt_id)

        with open(os.path.join(det_dir, "sr.json"), "w") as f:
            json.dump({
                "prompt_id": prompt_id,
                "prompt": prompt,
                "SR": metrics.sr,
                "confidence_total": result_data["confidence"],
                "confidence_detection": result_data["detection_confidence"],
                "confidence_vlm": result_data["vlm_confidence"],
                "confidence_memory": result_data["memory_confidence"]
            }, f, indent=2)

        with open(os.path.join(det_dir, "spl.json"), "w") as f:
            json.dump({
                "SPL": metrics.spl,
                "shortest_path_length": metrics.l_shortest,
                "actual_path_length": metrics.l_actual
            }, f, indent=2)

    def _save_summary_metrics(self):
        with open(os.path.join(self.episode_dir, "metrics.json"), "w") as f:
            json.dump(self.metrics_all, f, indent=2)

    def _map_callback(self, msg):
        if self.map is None:
            self.map = msg
            self.visualizer.set_map(msg)

    def _feedback_cb(self, feedback_msg):
        if self.debug:
            pass

    def _prepare_prompt_triplets(self):
        triplets = list(zip(
            self.prompts_train,
            self.prompts_eval,
            self.prompts_zero
        ))

        if self.shuffle_seed != 0:
            random.seed(self.shuffle_seed)
            random.shuffle(triplets)

        return triplets


def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorDashboard()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

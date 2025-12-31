#!/usr/bin/env python3
import os
import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from evaluator_msgs.srv import GetShortestPath
from sage_bt_msgs.srv import StartupCheck
from sage_bt_msgs.action import ExecutePrompt
from nav_msgs.msg import Path
from sage_datasets.utils import DatasetManager
from multimodal_query_msgs.msg import SemanticPrompt
from nav_msgs.msg import OccupancyGrid, Path
from sage_evaluator.utils.path_map_visualizer import PathMapVisualizer
from sage_evaluator.utils.metrics import Metrics
from sage_evaluator.utils.util import transform_path
import tf2_ros

class EvaluatorDashboard(Node):
    def __init__(self):
        super().__init__("evaluator_dashboard")

        # ---------------- Parameters ----------------
        self.declare_parameter("scene", "")
        self.declare_parameter("version", "")
        self.declare_parameter("episode_id", "")
        self.declare_parameter("debug_info", True)

        self.scene      = self.get_parameter("scene").value
        self.version    = self.get_parameter("version").value
        self.episode_id = self.get_parameter("episode_id").value
        self.debug      = self.get_parameter("debug_info").value

        # ---------------- Dataset loading ----------------
        self.dataset = DatasetManager(self.scene, self.version, self.episode_id)
        self.episode_dir = self.dataset.episode_dir()
        self.prompts_data = self.dataset.prompts()

        # Validate dataset
        if not all(k in self.prompts_data for k in ["train", "eval", "zero_shot"]):
            raise RuntimeError("prompts.json must contain 'train', 'eval', and 'zero_shot' keys.")

        self.prompts_train = self.prompts_data["train"]
        self.prompts_eval  = self.prompts_data["eval"]
        self.prompts_zero  = self.prompts_data["zero_shot"]

        if not (len(self.prompts_train) == len(self.prompts_eval) == len(self.prompts_zero)):
            raise RuntimeError("train/eval/zero_shot prompt lists must have equal length.")

        total_prompts = len(self.prompts_train)
        self.get_logger().info(f"Loaded {total_prompts} paired prompts from dataset.")
        self.get_logger().info(f"Episode directory: {self.episode_dir}")

        # ---------------- Service and action clients ----------------
        self.shortest_client = self.create_client(GetShortestPath, "/evaluator/get_shortest_path")
        self.startup_check_client = self.create_client(StartupCheck, "/sage_behaviour_tree/startup_check")
        self.bt_action_client = ActionClient(self, ExecutePrompt, "/sage_behaviour_tree/execute_prompt")

        # Publisher for the actual executed path
        self.executed_path_pub = self.create_publisher(Path, "/evaluator/executed_path", 10)

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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Wait for servers
        self._wait_for_servers()

        # Metrics store
        self.metrics_all = {}

        # Start evaluation
        self.run_evaluation()

    def _wait_for_servers(self):
        self.get_logger().info("Waiting for service/action servers...")

        self.shortest_client.wait_for_service()
        self.get_logger().info("Connected to /evaluator/get_shortest_path service.")
        self.startup_check_client.wait_for_service()
        self.get_logger().info("Connected to /sage_behaviour_tree/startup_check service.")
        self.bt_action_client.wait_for_server()
        self.get_logger().info("Connected to /sage_behaviour_tree/execute_prompt action server.")

        self.get_logger().info("All service/action servers are ready.")

    def run_evaluation(self):
        total = len(self.prompts_train)

        for i, (train_prompt, eval_prompt) in enumerate(zip(self.prompts_train,
                                                            self.prompts_eval),
                                                        start=1):

            zero_shot = self.prompts_zero[i - 1]

            self.get_logger().info("=" * 60)
            self.get_logger().info(f"[{i}/{total}] EVALUATING: {train_prompt}")
            self.get_logger().info("=" * 60)

            # Publish zero-shot
            zero_shot_msg = SemanticPrompt()
            zero_shot_msg.text_query = zero_shot
            # self.zero_shot_pub.publish(zero_shot_msg)
            self.get_logger().info(f"Published zero-shot prompt: \"{zero_shot}\"")

            # 1. Shortest path
            self.get_logger().info(f"Requesting shortest path for '{eval_prompt}'")
            shortest_path = self._call_shortest_path(eval_prompt)

            # 2. Startup check once
            if i == 1:
                if not self._call_startup_check():
                    self.get_logger().error("Startup check failed, aborting evaluation.")
                    return
                            
            # 3. Execute BT
            result_data = self._call_execute_prompt(train_prompt, zero_shot)
            if result_data is None:
                continue

            metrics = Metrics(self, result_data["success"], result_data["path"], shortest_path)
            self.metrics_all[train_prompt] = metrics.to_dict()

            plot_path = os.path.join(self.episode_dir, "detections", train_prompt, "paths.png")

            actual_path_transformed = transform_path(
                self.tf_buffer, "robot_original_pose_at_scan", result_data["path"]
            )

            self.visualizer.save_plot(
                plot_path,
                actual_path_transformed,
                shortest_path,
                actual_path_transformed.poses[0].pose,
                actual_path_transformed.poses[-1].pose,
                shortest_path.poses[-1].pose
            )

            self._save_prompt_metrics(train_prompt, metrics, result_data)

        # 6. Save episode summary
        self._save_summary_metrics()
        self.get_logger().info("All prompts completed. Evaluation finished.")

    def _call_shortest_path(self, query):
        req = GetShortestPath.Request()
        req.query = query

        future = self.shortest_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        if not resp or not resp.success:
            self.get_logger().warn(f"Shortest path for '{query}' failed.")
            return Path()

        self.get_logger().info(f"Shortest path received ({len(resp.path.poses)} poses, "
                               f"{resp.path_length:.2f} m)")
        return resp.path

    def _call_startup_check(self):
        req = StartupCheck.Request()

        future = self.startup_check_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None:
            self.get_logger().error("No response from startup_check.")
            return False

        self.get_logger().info(f"Startup check {'PASSED' if resp.ready else 'FAILED'}: {resp.report}")
        return resp.ready

    def _call_execute_prompt(self, query: str, zero_shot: str):
        goal = ExecutePrompt.Goal()
        goal.prompt = query
        goal.experiment_id = f"{self.scene}_{self.episode_id}"
        goal.timeout = 30.0
        goal.zero_shot_prompt = zero_shot

        det_dir = os.path.join(self.episode_dir, "detections", query)
        os.makedirs(det_dir, exist_ok=True)
        goal.save_directory = os.path.join(det_dir, "detection.png")

        self.get_logger().info(
            f"Executing BT for prompt '{query}' (zero-shot='{zero_shot}') → saving {goal.save_directory}"
        )

        send_future = self.bt_action_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("BT goal rejected.")
            return None

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        success = result.result
        confidence = result.confidence_score
        detection_confidence = result.detection_confidence
        vlm_confidence = result.vlm_confidence
        memory_confidence = result.memory_confidence
        actual_path = result.accumulated_path
        start_pose = result.start_pose
        end_pose = result.end_pose
        total_time = result.total_time

        # Publish path if available
        if actual_path and len(actual_path.poses) > 0:
            actual_path.header.stamp = self.get_clock().now().to_msg()
            if not actual_path.header.frame_id:
                actual_path.header.frame_id = "map"
            self.executed_path_pub.publish(actual_path)
            self.get_logger().info(f"Published executed path ({len(actual_path.poses)} poses)")

        self._save_detection_meta(query, success, "detection.png")

        self.get_logger().info(
            f"BT completed: success={success}, total_conf={confidence:.3f}, "
            f"det={detection_confidence:.3f}, vlm={vlm_confidence:.3f}, mem={memory_confidence:.3f}, "
            f"time={total_time:.2f}s"
        )

        return {
            "success": success,
            "confidence": confidence,
            "detection_confidence": detection_confidence,
            "vlm_confidence": vlm_confidence,
            "memory_confidence": memory_confidence,
            "path": actual_path,
            "start": start_pose,
            "end": end_pose,
            "total_time": total_time
        }

    def _save_detection_meta(self, prompt, success, image_name):
        det_dir = os.path.join(self.episode_dir, "detections", prompt)
        os.makedirs(det_dir, exist_ok=True)

        meta = {
            "object": prompt,
            "status": "SUCCESS" if success else "FAILURE",
            "image": image_name
        }

        with open(os.path.join(det_dir, "detection_meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

        self.get_logger().info(f"Saved detection meta → {det_dir}/detection_meta.json")

    def _feedback_cb(self, feedback_msg):
        if self.debug:
            fb = feedback_msg.feedback
            # self.get_logger().info(f"[BT] active_node={fb.active_node}, status={fb.status}")

    def _save_prompt_metrics(self, prompt: str, metrics: Metrics, result_data: dict):
        det_dir = os.path.join(self.episode_dir, "detections", prompt)
        os.makedirs(det_dir, exist_ok=True)

        # SR JSON including all confidence terms
        sr_path = os.path.join(det_dir, "sr.json")
        sr_data = {
            "SR": metrics.sr,
            "confidence_total": result_data["confidence"],
            "confidence_detection": result_data["detection_confidence"],
            "confidence_vlm": result_data["vlm_confidence"],
            "confidence_memory": result_data["memory_confidence"]
        }
        with open(sr_path, "w") as f:
            json.dump(sr_data, f, indent=2)

        # SPL JSON
        spl_path = os.path.join(det_dir, "spl.json")
        spl_data = {
            "SPL": metrics.spl,
            "shortest_path_length": metrics.l_shortest,
            "actual_path_length": metrics.l_actual
        }
        with open(spl_path, "w") as f:
            json.dump(spl_data, f, indent=2)

        self.get_logger().info(f"Saved metrics for '{prompt}' → {det_dir}")

    def _save_summary_metrics(self):
        summary_path = os.path.join(self.episode_dir, "metrics.json")
        with open(summary_path, "w") as f:
            json.dump(self.metrics_all, f, indent=2)
        self.get_logger().info(f"Saved episode summary → {summary_path}")

    def _map_callback(self, msg: OccupancyGrid):
        if self.map is None:
            self.map = msg
            self.visualizer.set_map(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EvaluatorDashboard()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

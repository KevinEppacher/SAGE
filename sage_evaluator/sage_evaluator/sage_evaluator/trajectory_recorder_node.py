#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np

from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from evaluator_msgs.msg import EvaluationEvent, EvaluationPathEvent

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        # Parameters
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("record_frequency", 5.0)
        self.declare_parameter("evaluation_event_sub", "/evaluator/dashboard/event")
        self.declare_parameter("iteration_status_sub", "/evaluator/dashboard/iteration_status")
        self.declare_parameter("path_event_pub", "path_event")

        self.map_frame = self.get_parameter("map_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.record_frequency = self.get_parameter("record_frequency").value
        self.evaluation_event_topic = self.get_parameter("evaluation_event_sub").value
        self.iteration_status_topic = self.get_parameter("iteration_status_sub").value
        self.path_event_topic = self.get_parameter("path_event_pub").value

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)

        # State
        self.current_trajectory = []
        self.current_prompt = ""
        self.active = False

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL       # <-- required
        qos.reliability = ReliabilityPolicy.RELIABLE

        # Publisher (DatasetWriter listens here)
        self.path_pub = self.create_publisher(
            EvaluationPathEvent,
            self.path_event_topic,
            qos
        )

        # Receive new evaluation event
        self.start_sub = self.create_subscription(
            EvaluationEvent,
            self.evaluation_event_topic,
            self.on_prompt_start,
            qos
        )

        # Receive BT completion messages
        self.status_sub = self.create_subscription(
            String,
            self.iteration_status_topic,
            self.on_prompt_end,
            qos
        )

        self.live_path_pub = self.create_publisher(Path, "live_path", 10)

        # Record robot path
        self.timer = self.create_timer(1.0 / self.record_frequency, self.record_step)

        self.get_logger().info("TrajectoryRecorder initialized")

    # -----------------------------------------------------------
    def on_prompt_start(self, msg: EvaluationEvent):
        """Start recording a new trajectory."""
        if not msg.current_prompt:
            self.get_logger().warn("Received EvaluationEvent without current_prompt")
            return

        self.current_prompt = msg.current_prompt
        self.current_trajectory = []
        self.active = True

        self.get_logger().info(f"Started trajectory for '{self.current_prompt}'")

    # -----------------------------------------------------------
    def on_prompt_end(self, msg: String):
        """Stop recording when BT reports completion."""
        if "'" not in msg.data:
            return

        finished_prompt = msg.data.split("'")[1]

        if finished_prompt != self.current_prompt:
            return

        self.get_logger().info(f"Ending trajectory for '{finished_prompt}'")

        self.active = False
        self.publish_actual_path()

    # -----------------------------------------------------------
    def record_step(self):
        if not self.active:
            return
        
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to lookup transform: {e}")
            return

        t = tf.transform.translation
        self.current_trajectory.append([t.x, t.y])
        self.publish_live_path()

    # -----------------------------------------------------------
    def compute_length(self):
        if len(self.current_trajectory) < 2:
            return 0.0

        arr = np.array(self.current_trajectory)
        return float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))

    # -----------------------------------------------------------
    def publish_actual_path(self):
        """Publishes EvaluationPathEvent to DatasetWriter."""
        if not self.current_trajectory:
            self.get_logger().warn("Empty trajectory → nothing to publish.")
            return

        msg = EvaluationPathEvent()
        msg.prompt = self.current_prompt
        msg.actual_length = self.compute_length()

        start = Point(x=self.current_trajectory[0][0], y=self.current_trajectory[0][1])
        end = Point(x=self.current_trajectory[-1][0], y=self.current_trajectory[-1][1])

        msg.start_pose = start
        msg.end_pose = end

        # Build nav_msgs/Path
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.current_trajectory:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        msg.trajectory = path
        msg.stamp = self.get_clock().now().to_msg()

        self.path_pub.publish(msg)
        self.get_logger().info(
            f"Published EvaluationPathEvent for '{self.current_prompt}' → length={msg.actual_length:.2f}m"
        )

    def publish_live_path(self):
        """Publishes the current trajectory as a live Path message."""
        if not self.current_trajectory:
            return

        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.current_trajectory:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.live_path_pub.publish(path)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import numpy as np
import tf_transformations
from openfusion_ros.slam import build_slam, BaseSLAM
from openfusion_ros.datasets import Dataset
from openfusion_ros.configs.build import get_config
from argparse import Namespace
from tqdm import tqdm
import argparse
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo

class TestDataset(Node):
    def __init__(self):
        super().__init__('test_dataset_publisher')

        self.rgb_pub = self.create_publisher(Image, '/rgb', 10)
        self.depth_pub = self.create_publisher(Image, '/depth', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        self.get_logger().info("Initializing...")

        parser = argparse.ArgumentParser()
        parser.add_argument('--algo', type=str, default="vlfusion")
        parser.add_argument('--vl', type=str, default="seem")
        parser.add_argument('--data', type=str, default="scannet")
        parser.add_argument('--scene', type=str, default="scene0010_01")
        parser.add_argument('--frames', type=int, default=-1)
        parser.add_argument('--device', type=str, default="cuda:0")
        parser.add_argument('--live', action='store_true')
        parser.add_argument('--stream', action='store_true')
        parser.add_argument('--save', action='store_true')
        parser.add_argument('--load', action='store_true')
        parser.add_argument('--host_ip', type=str, default="YOUR IP")
        args = parser.parse_args()

        params = get_config(args.data, args.scene)
        self.dataset: Dataset = params["dataset"](params["path"], args.frames, args.stream)
        intrinsic = self.dataset.load_intrinsics(params["img_size"], params["input_size"])
        self.slam: BaseSLAM = build_slam(args, intrinsic, params)

        self.rgb_list = []
        self.depth_list = []
        self.extrinsics_list = []
        self.intrinsic = intrinsic

        for rgb_path, depth_path, extrinsics in tqdm(self.dataset):
            rgb, depth = self.slam.io.from_file(rgb_path, depth_path)
            self.rgb_list.append(rgb)
            self.depth_list.append(depth)
            self.extrinsics_list.append(extrinsics)

        self.index = 0
        self.sim_time = 0.0  # Start bei 0 Sekunden
        self.time_step = 1.0 / 50.0  # 10 Hz (100 ms)

        self.timer = self.create_timer(self.time_step, self.timer_callback)
        self.get_logger().info(f"Loaded {len(self.rgb_list)} frames.")

    def to_ros_time(self, t: float) -> Time:
        secs = int(t)
        nsecs = int((t - secs) * 1e9)
        return Time(sec=secs, nanosec=nsecs)

    def timer_callback(self):
        if self.index >= len(self.rgb_list):
            self.get_logger().info("Restarting from frame 0.")
            self.index = 0
            self.sim_time = 0.0

        rgb = self.rgb_list[self.index]
        depth = self.depth_list[self.index]
        extrinsic = self.extrinsics_list[self.index]
        inverted_extrinsic = np.linalg.inv(extrinsic)

        ros_time = self.to_ros_time(self.sim_time)

        # Publish /clock
        clock_msg = Clock()
        clock_msg.clock = ros_time
        self.clock_pub.publish(clock_msg)

        # Publish RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding='bgr8')
        rgb_msg.header.stamp = ros_time
        rgb_msg.header.frame_id = 'camera'
        self.rgb_pub.publish(rgb_msg)

        # Publish Depth
        if depth.dtype != np.uint16:
            depth = (depth * 1000).astype(np.uint16)
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='16UC1')
        depth_msg.header.stamp = ros_time
        depth_msg.header.frame_id = 'camera'
        self.depth_pub.publish(depth_msg)

        # CameraInfo publizieren
        height, width = depth.shape
        cam_info_msg = self.create_camera_info_msg(ros_time, width, height, self.intrinsic)
        self.camera_info_pub.publish(cam_info_msg)

        # Publish TF: T_map_camera
        tf_msg = TransformStamped()
        tf_msg.header.stamp = ros_time
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'camera'

        tf_msg.transform.translation.x = inverted_extrinsic[0, 3]
        tf_msg.transform.translation.y = inverted_extrinsic[1, 3]
        tf_msg.transform.translation.z = inverted_extrinsic[2, 3]
        q = tf_transformations.quaternion_from_matrix(inverted_extrinsic)
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)

        self.get_logger().info(f"Published frame {self.index} at time {self.sim_time:.2f}s")

        self.index += 1
        self.sim_time += self.time_step

    def create_camera_info_msg(self, stamp, width, height, intrinsic):
        """Creates a CameraInfo message with given intrinsics and image size."""
        cam_info = CameraInfo()
        cam_info.header.stamp = stamp
        cam_info.header.frame_id = 'camera'
        cam_info.width = width
        cam_info.height = height

        cam_info.k = intrinsic.flatten().tolist()         # 3x3 intrinsic matrix
        cam_info.p = [  # Projection matrix (3x4), K extended with zeros
            intrinsic[0, 0], intrinsic[0, 1], intrinsic[0, 2], 0.0,
            intrinsic[1, 0], intrinsic[1, 1], intrinsic[1, 2], 0.0,
            intrinsic[2, 0], intrinsic[2, 1], intrinsic[2, 2], 0.0
        ]

        cam_info.d = []  # No distortion
        cam_info.distortion_model = "plumb_bob"
        cam_info.r = [1.0, 0.0, 0.0,  # identity matrix
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]

        return cam_info


def main(args=None):
    rclpy.init(args=args)
    node = TestDataset()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

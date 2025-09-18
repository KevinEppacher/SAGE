#!/usr/bin/env python3
# yoloe_rgb_cos_node.py
# ROS 2 Humble node: subscribe to /rgb, run YOLOE with open-vocab prompts,
# publish an annotated image and Detection2DArray with per-prompt cosine similarities.
#
# Run:
#   source /opt/ros/humble/setup.bash
#   pip install ultralytics opencv-python
#   sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs
#   python3 yoloe_rgb_cos_node.py
#
# Notes:
# - Code comments are in English, per your preference.
# - Cosine values are in [-1, 1]. For vision_msgs, we also map to [0, 1] via (cos+1)/2.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from geometry_msgs.msg import Pose2D

from cv_bridge import CvBridge
from ultralytics import YOLOE

import numpy as np
import torch
import torch.nn.functional as F


class YOLOERGBCosineNode(Node):
    def __init__(self):
        super().__init__("yoloe_rgb_cosine_node")

        # ---------------- Parameters ----------------
        self.declare_parameter("rgb_topic", "/rgb")
        self.declare_parameter("weights", "yoloe-11l-seg.pt")     # use *-seg for masks or *-.pt for boxes
        self.declare_parameter("prompts", ["chair", "door", "bed"])    # open-vocab text prompts
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("iou", 0.50)
        self.declare_parameter("publish_overlay", True)
        self.declare_parameter("publish_detections", True)

        p = self.get_parameter
        self.rgb_topic = p("rgb_topic").get_parameter_value().string_value
        self.weights = p("weights").get_parameter_value().string_value
        self.prompts = list(p("prompts").get_parameter_value().string_array_value)
        self.imgsz = int(p("imgsz").value)
        self.conf = float(p("conf").value)
        self.iou = float(p("iou").value)
        self.publish_overlay = bool(p("publish_overlay").value)
        self.publish_detections = bool(p("publish_detections").value)

        # ---------------- Model load ----------------
        self.model = YOLOE(self.weights).eval()
        # Get text prompt embeddings once and set classes (open-vocab)
        # --- Text-PE holen und korrekt an set_classes() übergeben ---
        raw_pe = self.model.get_text_pe(self.prompts)          # (1,K,D) oder (K,D)

        pe3 = raw_pe if raw_pe.dim() == 3 else raw_pe.unsqueeze(0)  # -> (1,K,D)

        self.model.set_classes(self.prompts, pe3)                   # API will 3D

        # Für Cosine: normierte 2D-Variante behalten
        txt2d = raw_pe.squeeze(0) if raw_pe.dim() == 3 else raw_pe  # -> (K,D)
        self.text_pe = F.normalize(txt2d, dim=-1)                   # (K,D), L2-normiert

        # Forward hook buffers for feature maps from the head/neck
        self._feat_maps = []
        self._hook = None
        self._attach_hook()

        self.bridge = CvBridge()

        # QoS tuned for camera streams
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(Image, self.rgb_topic, self.on_image, qos)

        if self.publish_overlay:
            self.pub_overlay = self.create_publisher(Image, "yoloe/overlay", 1)
        if self.publish_detections:
            self.pub_det = self.create_publisher(Detection2DArray, "yoloe/detections", 1)

        self.get_logger().info(f"YOLOE loaded: {self.weights}, prompts={self.prompts}, topic={self.rgb_topic}")

    # ---------------- Hook management ----------------

    def _attach_hook(self):
        """Attach a hook on the conv layer whose out_channels == D_txt (text embedding dim)."""
        self._feat_maps = []
        self._hook = None

        # Determine D_txt from current text embeddings
        raw_pe = self.model.get_text_pe(self.prompts)
        pe2 = raw_pe.squeeze(0) if raw_pe.dim() == 3 else raw_pe
        D_txt = int(pe2.shape[-1])

        target_modules = []
        for name, m in self.model.model.model.named_modules():
            if isinstance(m, torch.nn.Conv2d) and getattr(m, "out_channels", -1) == D_txt:
                target_modules.append((name, m))

        if not target_modules:
            self.get_logger().warn(f"No Conv2d with out_channels=={D_txt} found. Cosine will likely be unavailable.")
            # Fallback: last block
            target = self.model.model.model[-1]
        else:
            # Prefer the last one (closest to head)
            target = target_modules[-1][1]

        def head_hook(_m, _i, out):
            # Collect all 4D tensors and convert to (B,H,W,D)
            def _collect_4d(o, acc):
                if isinstance(o, torch.Tensor) and o.dim() == 4:
                    acc.append(o)
                elif isinstance(o, (list, tuple)):
                    for x in o: _collect_4d(x, acc)

            maps = []
            _collect_4d(out, maps)
            feats = []
            for t in maps:
                # (B,C,H,W) -> (B,H,W,D)
                if t.shape[1] != 1 and t.shape[1] != 3:
                    t = t.permute(0, 2, 3, 1).contiguous()
                else:
                    # if out is already (B,H,W,D), keep it
                    if t.shape[-1] not in (1, 3):
                        t = t.permute(0, 2, 3, 1).contiguous()
                feats.append(t.detach())
            if feats:
                self._feat_maps.append(feats)

        self._hook = target.register_forward_hook(head_hook)

    def _make_center(self, cx: float, cy: float):
        """Create a center submessage that matches the exact Pose2D type expected by BoundingBox2D.center."""
        center = type(BoundingBox2D().center)()  # construct the exact expected submessage type
        # Try common field layout
        if hasattr(center, "x"):
            center.x = float(cx)
            center.y = float(cy)
            if hasattr(center, "theta"):
                center.theta = 0.0
        elif hasattr(center, "position"):
            # Rare layout fallback
            center.position.x = float(cx)
            center.position.y = float(cy)
            if hasattr(center, "theta"):
                center.theta = 0.0
        return center

    def _clear_feats(self):
        self._feat_maps.clear()

    # ---------------- Feature sampling utils ----------------

    @staticmethod
    def _estimate_stride(H, W, ih, iw):
        """Estimate stride from image size to feature map size."""
        sx = iw / float(max(W, 1))
        sy = ih / float(max(H, 1))
        return 0.5 * (sx + sy)

    @staticmethod
    def _grid_sample_vec(feat_hw_d: torch.Tensor, cx: float, cy: float, stride: float, iw: int, ih: int):
        """
        Bilinear sample a D-dim vector from feature map at image-pixel center (cx, cy).
        feat_hw_d: (H, W, D) on same device as text_pe.
        Returns: (D,) tensor.
        """
        H, W, D = feat_hw_d.shape
        gx = cx / max(stride, 1e-6)
        gy = cy / max(stride, 1e-6)
        nx = (gx / max(W - 1, 1) - 0.5) * 2.0
        ny = (gy / max(H - 1, 1) - 0.5) * 2.0
        grid = torch.tensor([[[[nx, ny]]]], dtype=feat_hw_d.dtype, device=feat_hw_d.device)  # (1,1,1,2)
        feat_nchw = feat_hw_d.permute(2, 0, 1).unsqueeze(0)  # (1,D,H,W)
        samp = F.grid_sample(feat_nchw, grid, align_corners=True)  # (1,D,1,1)
        return samp.view(-1)  # (D,)

    # ---------------- ROS callback ----------------

    def on_image(self, msg: Image):
        # Convert ROS Image -> BGR
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")
            return

        # Inference (hook collects features)
        self._feat_maps.clear()
        try:
            res = self.model.predict(source=frame, imgsz=self.imgsz, conf=self.conf, iou=self.iou, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f"YOLOE inference failed: {e}")
            return

        # Overlay
        if self.publish_overlay:
            overlay = res.plot()
            out_img = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            out_img.header = msg.header
            self.pub_overlay.publish(out_img)

        if not self.publish_detections:
            return

        det_arr = Detection2DArray()
        det_arr.header = msg.header

        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            self.pub_det.publish(det_arr)
            return

        names = res.names
        def as_np1d(t): return t.detach().view(-1).cpu().numpy()
        xyxy    = boxes.xyxy.detach().cpu().numpy()
        confs   = as_np1d(boxes.conf)
        cls_ids = as_np1d(boxes.cls).astype(int) if boxes.cls is not None else np.full((len(boxes),), -1, int)

        # If no feature maps captured, publish conf-only
        if not self._feat_maps:
            self.get_logger().warn("No head features captured; cosine not available this frame.")
            self._publish_conf_only(det_arr, res, msg)
            self.pub_det.publish(det_arr)
            return

        # Filter maps to those with channel == D_txt
        K, D_txt = self.text_pe.shape
        pyr_all = [f[0] for f in self._feat_maps[0]]  # (H,W,D)
        cands = [Fm for Fm in pyr_all if Fm.shape[-1] == D_txt]

        if not cands:
            self.get_logger().warn(f"No feature maps with channel dim D={D_txt}; publishing conf-only.")
            self._publish_conf_only(det_arr, res, msg)
            self.pub_det.publish(det_arr)
            return

        # Sort by spatial size and keep top-3
        pyramid = sorted(cands, key=lambda t: t.shape[0]*t.shape[1], reverse=True)[:3]

        img_h, img_w = res.orig_shape
        strides = [self._estimate_stride(Fm.shape[0], Fm.shape[1], img_h, img_w) for Fm in pyramid]

        txt = self.text_pe.to(pyramid[0].device)  # (K,D)

        def choose_level(w, h):
            s_obj = max((w*h)**0.5, 1.0)
            diffs = [abs((img_w/Fm.shape[1]) - s_obj/32.0) for Fm in pyramid]
            return int(np.argmin(diffs))

        for i in range(len(boxes)):
            x1, y1, x2, y2 = xyxy[i]
            cx, cy = 0.5*(x1+x2), 0.5*(y1+y2)
            w, h = (x2-x1), (y2-y1)
            li = choose_level(w, h)

            # Sample object embedding
            Fm = pyramid[li].to(txt.device)                   # (H,W,D)
            emb = self._grid_sample_vec(Fm, cx, cy, strides[li], img_w, img_h)
            emb = F.normalize(emb.float(), dim=0)             # (D,)
            cos = (emb @ txt.T).detach().cpu().numpy()        # [-1,1]
            cos01 = 0.5*(cos+1.0)                             # [0,1]

            # Build Detection2D
            det = Detection2D()
            det.header = msg.header

            # main hypothesis (detector class+conf)
            cls_name = names.get(int(cls_ids[i]), str(int(cls_ids[i])))
            hyp_main = ObjectHypothesisWithPose()
            hyp_main.hypothesis.class_id = cls_name
            hyp_main.hypothesis.score = float(confs[i])
            det.results.append(hyp_main)

            # cosine per prompt
            for pj, prompt in enumerate(self.prompts):
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = f"cos:{prompt}"
                hyp.hypothesis.score = float(cos01[pj])
                det.results.append(hyp)

            # bbox with robust center creation
            bb = BoundingBox2D()
            center = self._make_center(cx, cy)
            bb.center = center
            bb.size_x = float(w)
            bb.size_y = float(h)
            det.bbox = bb

            det_arr.detections.append(det)

        self.pub_det.publish(det_arr)


    # ---------------- Helper: publish detections without cosine ----------------

    def _publish_conf_only(self, det_arr: Detection2DArray, res, msg):
        boxes = res.boxes
        if boxes is None or len(boxes) == 0:
            return
        names = res.names
        def as_np1d(t): return t.detach().view(-1).cpu().numpy()

        xyxy    = boxes.xyxy.detach().cpu().numpy()
        confs   = as_np1d(boxes.conf)
        cls_ids = as_np1d(boxes.cls).astype(int) if boxes.cls is not None else np.full((len(boxes),), -1, int)

        for i in range(len(boxes)):
            x1, y1, x2, y2 = xyxy[i]
            cx, cy = 0.5*(x1+x2), 0.5*(y1+y2)
            w, h = (x2-x1), (y2-y1)

            det = Detection2D()
            det.header = msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = names.get(int(cls_ids[i]), str(int(cls_ids[i])))
            hyp.hypothesis.score = float(confs[i])
            det.results.append(hyp)

            bb = BoundingBox2D()
            bb.center = self._make_center(cx, cy)   # robust center
            bb.size_x = float(w)
            bb.size_y = float(h)
            det.bbox = bb

            det_arr.detections.append(det)


def main():
    rclpy.init()
    node = YOLOERGBCosineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up hook explicitly
        try:
            if node._hook is not None:
                node._hook.remove()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""ROS 2 node that detects humans in a compressed image stream using
YOLO-FastestV2 on the NCNN runtime.

CPU-only (no GPU), ~0.5 MB model, tuned for small ARM boards such as the
Orange Pi 3 (quad Cortex-A53). Every setting lives in config/human_detector_yolo.yaml.

Needs the ncnn runtime:  pip install --no-deps ncnn
(the --no-deps avoids pulling in OpenCV 5.x, which shadows the ROS OpenCV and
breaks cv_bridge image conversion.)
"""

import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from vision_msgs.msg import BoundingBox2D, BoundingBox2DArray

PACKAGE = 'human_detector_yolo'

# Topic this node publishes its person boxes on.
TOPIC = 'detections/humans'
# Annotated frame (boxes + confidence), only drawn while someone subscribes.
DEBUG_TOPIC = 'human_detector/image'

# Model files under <share>/models (populated by scripts/download_model.sh).
DEFAULT_PARAM = 'yolo-fastestv2-opt.param'
DEFAULT_BIN = 'yolo-fastestv2-opt.bin'

# --- YOLO-FastestV2 specifics -------------------------------------------
INPUT_NAME = 'input.1'
OUTPUTS = ('794', '796')              # 22x22 (stride 16), 11x11 (stride 32)
ANCHORS = ([12.64, 19.39, 37.88, 51.48, 55.71, 138.31],
           [126.91, 78.23, 131.57, 214.55, 279.92, 258.87])
NMS_THRESH = 0.25
PERSON_CLASS = 0                      # "person" in the 80-class COCO model


class HumanDetectorNode(Node):

    def __init__(self):
        super().__init__('human_detector_yolo')
        self._bridge = CvBridge()
        self._share = get_package_share_directory(PACKAGE)

        # ---- input ---------------------------------------------------------
        self.image_topic = self.declare_parameter(
            'image_topic', '/camera/image_raw/compressed').value
        queue_size = self.declare_parameter('queue_size', 1).value
        # Cap detection rate (Hz). 0 = every frame. Extra frames are dropped
        # before the costly decode/inference, freeing the CPU.
        max_rate = float(self.declare_parameter('max_rate', 0.0).value)
        self._min_period = (1.0 / max_rate) if max_rate > 0 else 0.0
        self._last_proc = 0.0
        publish_debug = self.declare_parameter('publish_debug_image', True).value

        # ---- model / runtime ----------------------------------------------
        bin_path = self.declare_parameter('model_path', '').value
        param_path = self.declare_parameter('config_path', '').value
        self.num_threads = int(self.declare_parameter('num_threads', 4).value)
        size = int(self.declare_parameter('ncnn_input_size', 352).value)
        if size % 32 != 0:
            size = max(32, round(size / 32) * 32)
            self.get_logger().warning(
                f'ncnn_input_size must be a multiple of 32; using {size}')
        self.in_w = self.in_h = size
        use_fp16 = self.declare_parameter('ncnn_fp16', False).value
        self.threshold = float(self.declare_parameter(
            'confidence_threshold', 0.5).value)

        self._load_net(param_path, bin_path, use_fp16)

        self._pub = self.create_publisher(BoundingBox2DArray, TOPIC, 10)
        # Annotated frame; not advertised at all when disabled.
        self._debug_pub = (
            self.create_publisher(Image, DEBUG_TOPIC, 10) if publish_debug else None)

        self._sub = self.create_subscription(
            CompressedImage, self.image_topic, self._image_cb, queue_size)

        self.get_logger().info(
            f"human_detector_yolo ready: input={self.in_w}x{self.in_h}, "
            f"threads={self.num_threads}, max_rate={max_rate}Hz, "
            f"confidence_threshold={self.threshold}, "
            f"subscribing to '{self.image_topic}', publishing '{TOPIC}', "
            f"debug image {'on ' + DEBUG_TOPIC if publish_debug else 'disabled'}")

    # -----------------------------------------------------------------------
    def _load_net(self, param_path, bin_path, use_fp16):
        try:
            import ncnn
        except ImportError as exc:
            self.get_logger().fatal(
                "ncnn backend needs the 'ncnn' package: pip install --no-deps ncnn")
            raise RuntimeError('ncnn runtime missing') from exc

        param_path = param_path or os.path.join(
            self._share, 'models', DEFAULT_PARAM)
        bin_path = bin_path or os.path.join(self._share, 'models', DEFAULT_BIN)
        for p in (param_path, bin_path):
            if not os.path.isfile(p):
                self.get_logger().fatal(
                    f"model not found at '{p}'. Run scripts/download_model.sh.")
                raise RuntimeError('model file missing')

        self._ncnn = ncnn
        self._net = ncnn.Net()
        self._net.opt.num_threads = self.num_threads   # set BEFORE load
        if use_fp16:                                   # may help on ARM (A55+)
            self._net.opt.use_fp16_packed = True
            self._net.opt.use_fp16_storage = True
        self._net.load_param(param_path)
        self._net.load_model(bin_path)
        self.get_logger().info(
            f"loaded YOLO-FastestV2 '{bin_path}' "
            f"(input {self.in_w}x{self.in_h}, fp16={use_fp16})")

    # -----------------------------------------------------------------------
    def _image_cb(self, msg: CompressedImage):
        # Rate limit: drop the frame before the costly decode/inference.
        if self._min_period > 0.0:
            now = time.monotonic()
            if now - self._last_proc < self._min_period:
                return
            self._last_proc = now

        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'failed to decode image: {exc}')
            return

        h, w = frame.shape[:2]
        out = BoundingBox2DArray()
        out.header = msg.header
        # Skip drawing/encoding on the board unless enabled and someone watches.
        debug = self._debug_pub is not None and self._debug_pub.get_subscription_count() > 0

        for confidence, fx1, fy1, fx2, fy2 in self._infer(frame):
            x1 = max(0, min(w - 1, int(fx1)))
            y1 = max(0, min(h - 1, int(fy1)))
            x2 = max(0, min(w, int(fx2)))
            y2 = max(0, min(h, int(fy2)))
            if x2 <= x1 or y2 <= y1:
                continue

            box = BoundingBox2D()
            # vision_msgs boxes are center-based; convert from the top-left rect.
            box.center.position.x = float(x1 + (x2 - x1) / 2.0)
            box.center.position.y = float(y1 + (y2 - y1) / 2.0)
            box.size_x = float(x2 - x1)
            box.size_y = float(y2 - y1)

            out.boxes.append(box)

            if debug:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f'{confidence:.2f}', (x1, max(12, y1 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        self._pub.publish(out)

        if debug:
            img = self._bridge.cv2_to_imgmsg(frame, 'bgr8')
            img.header = msg.header
            self._debug_pub.publish(img)

    # -----------------------------------------------------------------------
    def _infer(self, frame):
        """Run YOLO-FastestV2 -> list of (conf, x1, y1, x2, y2) for persons."""
        h, w = frame.shape[:2]
        m = self._ncnn.Mat.from_pixels_resize(
            frame, self._ncnn.Mat.PixelType.PIXEL_BGR, w, h, self.in_w, self.in_h)
        m.substract_mean_normalize([0.0, 0.0, 0.0], [1 / 255.0] * 3)
        ex = self._net.create_extractor()
        ex.input(INPUT_NAME, m)
        outs = [np.array(ex.extract(name)[1]) for name in OUTPUTS]

        sw, sh = w / self.in_w, h / self.in_h
        raw = []
        for i, arr in enumerate(outs):            # arr: (gridH, gridW, 95)
            gh, gw, _ = arr.shape
            stride = self.in_h // gh
            anch = ANCHORS[i]
            # Vectorised: per-cell best class (shared across anchors), then only
            # loop the few cells/anchors that clear the confidence threshold.
            reg = arr[:, :, :12].reshape(gh, gw, 3, 4)   # [y, x, anchor, (x,y,w,h)]
            obj = arr[:, :, 12:15]                       # objectness per anchor
            cls = arr[:, :, 15:95]
            ca = cls.argmax(2)
            cs = cls.max(2)
            for b in range(3):                           # 3 anchors
                score = obj[:, :, b] * cs
                ys, xs = np.where(score > self.threshold)
                for yy, xx in zip(ys.tolist(), xs.tolist()):
                    if int(ca[yy, xx]) != PERSON_CLASS:
                        continue
                    rx, ry, rw, rh = reg[yy, xx, b]
                    bcx = ((rx * 2 - 0.5) + xx) * stride
                    bcy = ((ry * 2 - 0.5) + yy) * stride
                    bw = (rw * 2) ** 2 * anch[b * 2]
                    bh = (rh * 2) ** 2 * anch[b * 2 + 1]
                    raw.append((float(score[yy, xx]),
                                (bcx - bw / 2) * sw, (bcy - bh / 2) * sh,
                                (bcx + bw / 2) * sw, (bcy + bh / 2) * sh))

        # YOLO has no built-in NMS -> dedupe the person boxes.
        if not raw:
            return []
        bbs = [[r[1], r[2], r[3] - r[1], r[4] - r[2]] for r in raw]
        scs = [r[0] for r in raw]
        keep = cv2.dnn.NMSBoxes(bbs, scs, self.threshold, NMS_THRESH)
        return [raw[int(k)] for k in np.array(keep).flatten()]


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HumanDetectorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

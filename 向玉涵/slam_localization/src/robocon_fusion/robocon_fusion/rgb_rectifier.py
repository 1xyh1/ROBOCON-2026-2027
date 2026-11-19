"""Undistort RGB with captured CameraInfo; preserve measurement stamps exactly."""
import json
import copy
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from robocon_core.livo_config import camera_model


class RgbRectifier(Node):
    def __init__(self):
        super().__init__("livo_rgb_rectifier")
        path = self.declare_parameter("camera_file", "").value
        self.info = json.loads(Path(path).read_text())["camera_info"]
        K, D, R, projection = camera_model(self.info)
        self.maps = cv2.initUndistortRectifyMap(
            K, D, R, projection, (self.info["width"], self.info["height"]), cv2.CV_16SC2)
        self.bridge = CvBridge()
        # The owned RGB gate/rectifier pipeline offers RELIABLE; preserve large
        # image fragments rather than silently losing frames with BEST_EFFORT.
        image_qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.RELIABLE)
        self.output = self.create_publisher(Image, "/fusion/input/color_rect", image_qos)
        # This topic already passed the exact RGB-D/metadata timestamp gate during capture.
        self.subscription = self.create_subscription(
            Image, "/sensors/d435i/color/image_raw", self.on_image, image_qos)
        self.accepted = 0
        self.rejected = 0

    def on_image(self, image):
        if ((image.width, image.height) != (self.info["width"], self.info["height"])
                or image.header.frame_id != self.info["frame_id"]):
            self.rejected += 1
            self.get_logger().error("Image dimensions/frame differ from captured calibration")
            return
        raw = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        rectified = cv2.remap(raw, *self.maps, interpolation=cv2.INTER_LINEAR)
        output = self.bridge.cv2_to_imgmsg(np.ascontiguousarray(rectified), encoding="bgr8")
        output.header = copy.deepcopy(image.header)
        # A non-identity CameraInfo.R rotates the optical coordinate system.
        output.header.frame_id = "livo_color_rect_optical_frame"
        self.output.publish(output)
        self.accepted += 1


def main(args=None):
    rclpy.init(args=args)
    node = RgbRectifier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print(f"RGB rectifier accepted={node.accepted}, rejected={node.rejected}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

"""Detect a coloured object and publish camera-frame angular bounds.

Lab partners: TODO_NAME_1, TODO_NAME_2 (replace before submission).
All parameters are validated at startup and read-only while the node runs.
"""

import copy
import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image

from chase_object_interfaces.msg import ObjectBearing
from .detection import (
    CameraModel, DetectorConfig, annotate_detection, detect_target, detection_bearings,
)


class DetectObject(Node):
    def __init__(self):
        super().__init__("detect_object")
        defaults = {
            "image_topic": "/camera/image_raw/compressed",
            "image_transport": "compressed",
            "camera_info_topic": "/camera/camera_info",
            "bearing_topic": "/object/bearing",
            "debug_image_topic": "/object/debug_image",
            "camera_frame": "camera_optical_frame",
            "hsv_lower": [35, 70, 60],
            "hsv_upper": [85, 255, 255],
            "min_area": 150.0,
            "morph_kernel": 3,
            "horizontal_fov_deg": 62.2,
        }
        self._params = {}
        for name, value in defaults.items():
            self._params[name] = self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True),
            ).value
        for name in ("image_topic", "camera_info_topic", "bearing_topic", "debug_image_topic", "camera_frame"):
            if not isinstance(self._params[name], str) or not self._params[name].strip():
                raise ValueError(f"{name} must be a non-empty string")
        transport = self._params["image_transport"]
        if transport not in ("compressed", "raw"):
            raise ValueError("image_transport must be 'compressed' or 'raw'")
        fov = self._params["horizontal_fov_deg"]
        if not math.isfinite(fov) or not 0 < fov < 180:
            raise ValueError("horizontal_fov_deg must be between 0 and 180")
        self._config = DetectorConfig(
            tuple(self._params["hsv_lower"]), tuple(self._params["hsv_upper"]),
            self._params["min_area"], self._params["morph_kernel"],
        )
        self._bridge = CvBridge()
        self._camera_info = None
        self._warnings = set()
        self._publisher = self.create_publisher(ObjectBearing, self._params["bearing_topic"], 10)
        self._debug_publisher = self.create_publisher(Image, self._params["debug_image_topic"], 1)
        self._camera_info_subscription = self.create_subscription(
            CameraInfo, self._params["camera_info_topic"], self._on_camera_info,
            qos_profile_sensor_data,
        )
        self._image_subscription = self.create_subscription(
            CompressedImage if transport == "compressed" else Image,
            self._params["image_topic"], self._on_image, qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Detecting HSV target on {self._params['image_topic']} ({transport}); "
            "bearing is positive to the LEFT of the optical axis."
        )

    def _warn_once(self, key, text):
        if key not in self._warnings:
            self._warnings.add(key)
            self.get_logger().warning(text)

    def _on_camera_info(self, message):
        self._camera_info = message

    def _camera_model(self, width, height, frame_id):
        info = self._camera_info
        if info is not None:
            try:
                if info.header.frame_id and frame_id and info.header.frame_id != frame_id:
                    raise ValueError("image and CameraInfo frame IDs differ")
                # Cropping/binning changes pixel coordinates; reject instead of guessing.
                if (
                    info.binning_x > 1 or info.binning_y > 1
                    or info.roi.x_offset or info.roi.y_offset
                    or info.roi.width not in (0, info.width)
                    or info.roi.height not in (0, info.height)
                ):
                    raise ValueError("binned/cropped CameraInfo is unsupported; provide matching adjusted intrinsics")
                model = CameraModel.from_calibration(
                    width, height, info.width, info.height, info.k, info.d, info.distortion_model,
                )
                if width != info.width or height != info.height:
                    self._warn_once(
                        "scaled_calibration", "Image resolution differs from CameraInfo: scaling K. "
                        "This assumes a resize of the same view; cropped images need adjusted calibration.",
                    )
                return model
            except (ValueError, TypeError) as error:
                self._warn_once("bad_calibration", f"CameraInfo cannot be used: {error}")
        self._warn_once(
            "fov_fallback", "No usable CameraInfo; bearings are APPROXIMATE using "
            f"horizontal_fov_deg={self._params['horizontal_fov_deg']}. "
            "Publish calibrated CameraInfo for accurate LiDAR association.",
        )
        return CameraModel.from_fov(width, height, self._params["horizontal_fov_deg"])

    def _on_image(self, message):
        result = ObjectBearing()
        result.header = copy.deepcopy(message.header)
        if not result.header.frame_id:
            result.header.frame_id = self._params["camera_frame"]
            self._warn_once("empty_frame", "Image frame_id is empty; using configured camera_frame.")
        # A failed decode/absent target is explicitly invalid; downstream must stop.
        result.detected = False
        result.bearing = result.angle_min = result.angle_max = result.confidence = 0.0
        image = None
        detection = None
        try:
            if self._params["image_transport"] == "compressed":
                image = cv2.imdecode(np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("compressed image decode failed")
            else:
                image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            detection = detect_target(image, self._config)
            if detection is not None:
                height, width = image.shape[:2]
                camera = self._camera_model(width, height, result.header.frame_id)
                bearing, angle_min, angle_max = detection_bearings(detection, camera)
                if not all(math.isfinite(v) for v in (bearing, angle_min, angle_max, detection.confidence)):
                    raise ValueError("non-finite detection result")
                result.bearing, result.angle_min, result.angle_max = bearing, angle_min, angle_max
                result.confidence = detection.confidence
                result.detected = True
        except Exception as error:
            self._warn_once("processing_error", f"Image processing failed; publishing invalid target: {error}")
        self._publisher.publish(result)
        if image is not None and self._debug_publisher.get_subscription_count() > 0:
            try:
                debug = self._bridge.cv2_to_imgmsg(annotate_detection(image, detection), encoding="bgr8")
                debug.header = copy.deepcopy(result.header)
                self._debug_publisher.publish(debug)
            except Exception as error:
                self._warn_once("debug_error", f"Cannot publish debug image: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DetectObject()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

from __future__ import annotations

from pathlib import Path
from typing import Any
import time
import yaml

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class UsbCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('usb_camera_node')

        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('camera_name', 'overhead_usb_cam')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('publish_rectified', True)

        self.video_device = str(self.get_parameter('video_device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.calibration_file = str(self.get_parameter('calibration_file').value)
        self.publish_rectified = bool(self.get_parameter('publish_rectified').value)

        self.bridge = CvBridge()

        self.raw_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.rect_pub = self.create_publisher(Image, '/camera/image_rect', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.camera_info = self._load_camera_info(self.calibration_file)
        self.k = np.array(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        self.d = np.array(self.camera_info.d, dtype=np.float64)
        # Precompute undistort maps once (cv2.undistort recomputes these every
        # frame, which is the dominant cost at 1080p). Use cv2.remap with these
        # maps for a 5-10x speedup.
        self._undistort_map1 = None
        self._undistort_map2 = None
        if len(self.d) > 0:
            self._undistort_map1, self._undistort_map2 = cv2.initUndistortRectifyMap(
                self.k, self.d, None, self.k,
                (self.width, self.height), cv2.CV_16SC2)

        self.capture = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        # Request MJPG before setting size: many UVC cams only offer high
        # resolutions at full FPS in MJPG (YUYV is often 5-10 fps at 1080p).
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        # Keep only the newest frame in the driver buffer; otherwise frames
        # accumulate when the processing loop falls behind real-time.
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_w = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)
        if (actual_w, actual_h) != (self.width, self.height):
            self.get_logger().warn(
                f'Requested {self.width}x{self.height} but camera negotiated '
                f'{actual_w}x{actual_h} @ {actual_fps:.1f} FPS')

        if not self.capture.isOpened():
            raise RuntimeError(f'Could not open camera device: {self.video_device}')

        timer_period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(timer_period, self._tick)
        self.get_logger().info(f'Opened USB camera {self.video_device} at {self.width}x{self.height} @ {self.fps:.1f} FPS')

    def _load_camera_info(self, calibration_file: str) -> CameraInfo:
        msg = CameraInfo()
        msg.header.frame_id = self.frame_id
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'
        msg.k = [600.0, 0.0, self.width / 2.0,
                 0.0, 600.0, self.height / 2.0,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [600.0, 0.0, self.width / 2.0, 0.0,
                 0.0, 600.0, self.height / 2.0, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        if not calibration_file:
            self.get_logger().warn('No calibration file provided. Using placeholder intrinsics.')
            return msg

        path = Path(calibration_file)
        if not path.exists():
            self.get_logger().warn(f'Calibration file not found: {calibration_file}. Using placeholder intrinsics.')
            return msg

        with path.open('r', encoding='utf-8') as f:
            data: dict[str, Any] = yaml.safe_load(f) or {}

        msg.width = int(data.get('image_width', self.width))
        msg.height = int(data.get('image_height', self.height))
        msg.distortion_model = data.get('distortion_model', 'plumb_bob')
        msg.d = list(data.get('distortion_coefficients', {}).get('data', msg.d))
        msg.k = list(data.get('camera_matrix', {}).get('data', msg.k))
        msg.r = list(data.get('rectification_matrix', {}).get('data', msg.r))
        msg.p = list(data.get('projection_matrix', {}).get('data', msg.p))
        self.get_logger().info(f'Loaded camera calibration from {calibration_file}')
        return msg

    def _tick(self) -> None:
        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.get_logger().warning('Failed to read frame from USB camera.')
            return

        stamp = self.get_clock().now().to_msg()

        raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = self.camera_info.width
        info_msg.height = self.camera_info.height
        info_msg.distortion_model = self.camera_info.distortion_model
        info_msg.d = list(self.camera_info.d)
        info_msg.k = list(self.camera_info.k)
        info_msg.r = list(self.camera_info.r)
        info_msg.p = list(self.camera_info.p)

        self.raw_pub.publish(raw_msg)
        self.info_pub.publish(info_msg)

        if self.publish_rectified:
            rect = self._rectify(frame)
            rect_msg = self.bridge.cv2_to_imgmsg(rect, encoding='bgr8')
            rect_msg.header.stamp = stamp
            rect_msg.header.frame_id = self.frame_id
            self.rect_pub.publish(rect_msg)

    def _rectify(self, frame: np.ndarray) -> np.ndarray:
        if self._undistort_map1 is None:
            return frame
        try:
            return cv2.remap(frame, self._undistort_map1, self._undistort_map2,
                             interpolation=cv2.INTER_LINEAR)
        except cv2.error as exc:
            self.get_logger().warning(f'OpenCV remap failed: {exc}. Publishing raw image on image_rect semantics.')
            return frame

    def destroy_node(self) -> bool:
        if hasattr(self, 'capture'):
            self.capture.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

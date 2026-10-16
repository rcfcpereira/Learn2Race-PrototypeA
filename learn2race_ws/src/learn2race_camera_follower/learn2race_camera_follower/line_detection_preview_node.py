#!/usr/bin/env python3
"""Lightweight detection preview, viewable from any device with a browser.

The original preview published a full-resolution raw ``bgr8`` image on every
frame whether or not anyone was watching. At 640 x 480 that is roughly 920 kB a
frame, about 14 MB/s at 15 fps, which no wireless link in a teaching laboratory
will carry, and it re-ran the detector on the SBC for nobody's benefit.

This version does three things differently.

* **It does nothing until someone is watching.** With no ROS subscriber and no
  browser connected, frames are dropped before any processing happens.
* **It sends much less.** Frames are downscaled, JPEG-compressed once, and rate
  limited, since a person watching needs a few frames a second, not thirty.
* **It needs no ROS on the viewing device.** A built-in MJPEG server makes the
  stream available at ``http://<robot>:8080/`` to a phone or laptop browser. The
  same JPEG is also published as a standard ``CompressedImage`` for RViz, rqt
  and Foxglove.

It can also overlay the heading from the orientation filter and the compass, so
that the IMU and magnetometer integration can be checked by eye while turning
the robot, without a second tool.
"""

# References (full entries: docs/references.md at the repository root):
#   [R30] OpenCV 4.x: drawing and JPEG encoding
#   [R20] cv_bridge: cv_bridge
# More details: README.md of this package; preview_server.py

from __future__ import annotations

import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image, Imu, MagneticField

from .detector import DetectorConfig, LineDetector, draw_preview

INDEX_HTML = b"""<!doctype html><html><head><meta name="viewport"
content="width=device-width,initial-scale=1"><title>Learn2Race preview</title>
<style>body{margin:0;background:#111;color:#ddd;font:14px sans-serif;
text-align:center}img{max-width:100%;height:auto}</style></head><body>
<p>Learn2Race line detection preview</p><img src="/stream.mjpg"></body></html>"""


class _FrameBuffer:
    """The latest encoded frame, shared between the node and HTTP clients."""

    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg: bytes | None = None
        self.clients = 0

    def publish(self, jpeg: bytes) -> None:
        with self.condition:
            self.jpeg = jpeg
            self.condition.notify_all()


def _make_handler(buffer: _FrameBuffer):

    class Handler(BaseHTTPRequestHandler):

        def log_message(self, *_args):          # keep the robot log clean
            pass

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.end_headers()
                self.wfile.write(INDEX_HTML)
                return

            if self.path != '/stream.mjpg':
                self.send_error(404)
                return

            self.send_response(200)
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()

            with buffer.condition:
                buffer.clients += 1
            try:
                while True:
                    with buffer.condition:
                        buffer.condition.wait(timeout=2.0)
                        frame = buffer.jpeg
                    if frame is None:
                        continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(frame)}\r\n\r\n'.encode())
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with buffer.condition:
                    buffer.clients -= 1

    return Handler


class LineDetectionPreviewNode(Node):

    def __init__(self) -> None:
        super().__init__('line_detection_preview_node')

        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('preview_topic', 'line_preview/image_raw/compressed')
        self.declare_parameter('preview_width', 320)
        self.declare_parameter('preview_hz', 5.0)
        self.declare_parameter('jpeg_quality', 60)
        self.declare_parameter('http_enabled', True)
        self.declare_parameter('http_port', 8080)
        self.declare_parameter('overlay_heading', True)

        for name, default in (('roi_top_ratio', 0.30), ('roi_bottom_ratio', 0.95),
                              ('black_threshold', 70), ('min_contour_area_ratio', 0.003),
                              ('target_line_distance_mm', 50.0),
                              ('image_start_distance_mm', 70.0),
                              ('image_depth_mm', 220.0), ('target_row_band_px', 6),
                              ('curve_lookahead_px', 18), ('curve_gain', 0.55),
                              ('max_line_width_ratio', 0.45)):
            self.declare_parameter(name, default)

        self.width = max(80, int(self.get_parameter('preview_width').value))
        self.period = 1.0 / max(0.5, float(self.get_parameter('preview_hz').value))
        self.quality = int(self.get_parameter('jpeg_quality').value)
        self.overlay = bool(self.get_parameter('overlay_heading').value)

        self.detector = LineDetector(DetectorConfig(
            roi_top_ratio=float(self.get_parameter('roi_top_ratio').value),
            roi_bottom_ratio=float(self.get_parameter('roi_bottom_ratio').value),
            black_threshold=int(self.get_parameter('black_threshold').value),
            min_contour_area_ratio=float(self.get_parameter('min_contour_area_ratio').value),
            target_line_distance_mm=float(self.get_parameter('target_line_distance_mm').value),
            image_start_distance_mm=float(self.get_parameter('image_start_distance_mm').value),
            image_depth_mm=float(self.get_parameter('image_depth_mm').value),
            target_row_band_px=int(self.get_parameter('target_row_band_px').value),
            curve_lookahead_px=int(self.get_parameter('curve_lookahead_px').value),
            curve_gain=float(self.get_parameter('curve_gain').value),
            max_line_width_ratio=float(self.get_parameter('max_line_width_ratio').value),
        ))
        self.bridge = CvBridge()
        self.last_sent = 0.0
        self.imu_yaw: float | None = None
        self.mag_heading: float | None = None

        cv2.setNumThreads(1)

        sensor_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                reliability=ReliabilityPolicy.BEST_EFFORT,
                                durability=DurabilityPolicy.VOLATILE)

        self.publisher = self.create_publisher(
            CompressedImage, str(self.get_parameter('preview_topic').value),
            sensor_qos)
        self.create_subscription(Image, str(self.get_parameter('image_topic').value),
                                 self._on_image, sensor_qos)
        if self.overlay:
            self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
            self.create_subscription(MagneticField, 'imu/mag', self._on_mag, 10)

        self.buffer = _FrameBuffer()
        if bool(self.get_parameter('http_enabled').value):
            port = int(self.get_parameter('http_port').value)
            self.server = ThreadingHTTPServer(('0.0.0.0', port),
                                              _make_handler(self.buffer))
            threading.Thread(target=self.server.serve_forever, daemon=True).start()
            self.get_logger().info(
                f'Preview at http://<robot-address>:{port}/ '
                f'({self.width} px, {1.0 / self.period:.0f} Hz, JPEG q{self.quality})')

    # ------------------------------------------------------------ orientation

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        if q.w == 0.0 and q.x == 0.0 and q.y == 0.0 and q.z == 0.0:
            return                               # no orientation estimate yet
        self.imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_mag(self, msg: MagneticField) -> None:
        # Uncompensated for tilt: valid only with the robot level, which is the
        # case on a flat track. Compared against the filter yaw it shows whether
        # the two agree, which is the point of the overlay.
        self.mag_heading = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)

    # ----------------------------------------------------------------- frames

    def _watched(self) -> bool:
        return self.publisher.get_subscription_count() > 0 or self.buffer.clients > 0

    def _on_image(self, msg: Image) -> None:
        # The cheapest possible check first: with nobody watching, stop here.
        if not self._watched():
            return
        now = time.monotonic()
        if now - self.last_sent < self.period:
            return
        self.last_sent = now

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().warn(f'image conversion failed: {exc}',
                                   throttle_duration_sec=5.0)
            return
        if bgr is None or bgr.size == 0:
            return

        # Downscale before detection as well as before encoding: the overlay
        # only needs to be approximately right, and the SBC cost halves.
        scale = self.width / float(bgr.shape[1])
        if scale < 1.0:
            bgr = cv2.resize(bgr, (self.width, int(bgr.shape[0] * scale)),
                             interpolation=cv2.INTER_AREA)

        result = self.detector.detect(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        preview = draw_preview(bgr, result)
        if self.overlay:
            self._draw_heading(preview)

        ok, encoded = cv2.imencode('.jpg', preview,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        jpeg = encoded.tobytes()

        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = jpeg
        self.publisher.publish(out)
        self.buffer.publish(jpeg)

    def _draw_heading(self, image: np.ndarray) -> None:
        height = image.shape[0]
        lines = []
        if self.imu_yaw is not None:
            lines.append(f'filter yaw {math.degrees(self.imu_yaw):+7.1f} deg')
        if self.mag_heading is not None:
            lines.append(f'compass    {math.degrees(self.mag_heading):+7.1f} deg')
        if self.imu_yaw is not None and self.mag_heading is not None:
            diff = math.degrees(math.atan2(math.sin(self.imu_yaw - self.mag_heading),
                                           math.cos(self.imu_yaw - self.mag_heading)))
            lines.append(f'difference {diff:+7.1f} deg')
        for i, text in enumerate(lines):
            cv2.putText(image, text, (8, height - 10 - 18 * (len(lines) - 1 - i)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)


def main() -> None:
    rclpy.init()
    node = LineDetectionPreviewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'server'):
            node.server.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

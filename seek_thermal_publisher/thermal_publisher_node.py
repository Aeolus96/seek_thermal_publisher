#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

from seekcamera import (  # type: ignore
    SeekCameraManager,
    SeekCameraManagerEvent,
    SeekCameraFrameFormat,
    SeekCameraIOType,
)


class SeekThermalPublisher(Node):
    def __init__(self):
        super().__init__("seek_thermal_publisher")

        # Parameters
        self.declare_parameter("topic_raw", "/seek/radiometric_raw")  # raw radiometry: mono16 or 32FC1 [ROS encodings]
        self.declare_parameter("topic_view", "/seek/auto_scaled")  # white-hot preview: mono8 [ROS encodings]
        self.declare_parameter("frame_id", "seek_camera_optical")
        self.declare_parameter("percentile_clip", 2.0)  # 0.0 for min/max; e.g., 2.0 clips 2% low/high

        self.topic_raw = self.get_parameter("topic_raw").get_parameter_value().string_value
        self.topic_view = self.get_parameter("topic_view").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.pclip = float(self.get_parameter("percentile_clip").get_parameter_value().double_value)

        self.pub_raw = self.create_publisher(Image, self.topic_raw, 10)
        self.pub_view = self.create_publisher(Image, self.topic_view, 10)

        # Prefer thermography fixed-point, then float, finally grayscale (display only)
        if hasattr(SeekCameraFrameFormat, "THERMOGRAPHY_FIXED_10_6"):
            self._requested_format = SeekCameraFrameFormat.THERMOGRAPHY_FIXED_10_6
            self._mode = "fixed"
        elif hasattr(SeekCameraFrameFormat, "THERMOGRAPHY_FLOAT"):
            self._requested_format = SeekCameraFrameFormat.THERMOGRAPHY_FLOAT
            self._mode = "float"
        else:
            self._requested_format = SeekCameraFrameFormat.GRAYSCALE
            self._mode = "gray"

        self._camera = None
        self._manager = SeekCameraManager(SeekCameraIOType.USB)  # USB discovery per bindings
        self._manager.register_event_callback(self._on_event, None)
        self.get_logger().info(f"Ready; will request format: {self._requested_format}")

    def destroy_node(self):
        try:
            if self._camera is not None:
                try:
                    self._camera.capture_session_stop()
                except Exception:
                    pass
                try:
                    self._camera.unregister_frame_available_callback(self._on_frame)
                except Exception:
                    pass
                self._camera = None
        except Exception as exc:
            self.get_logger().warning(f"Cleanup error: {exc}", throttle_duration_sec=2.0)
        return super().destroy_node()

    def _on_event(self, camera, event_type, event_status, _user):
        try:
            if event_type == SeekCameraManagerEvent.CONNECT and self._camera is None:
                self._camera = camera
                self._camera.register_frame_available_callback(self._on_frame, None)
                self._camera.capture_session_start(self._requested_format)
                self.get_logger().info(f"Capture session started with {self._requested_format}")
            elif event_type == SeekCameraManagerEvent.DISCONNECT and self._camera == camera:
                try:
                    camera.capture_session_stop()
                except Exception:
                    pass
                self._camera = None
                self.get_logger().warning("Camera disconnected")
            else:
                self.get_logger().warning(
                    f"Camera event: {event_type} status: {event_status}", throttle_duration_sec=2.0
                )
        except Exception as exc:
            self.get_logger().error(f"Event handler error: {exc}")

    def _on_frame(self, _camera, camera_frame, _user):
        try:
            if self._mode == "fixed":
                for name in ("thermography_fixed_10_6", "thermometry_fixed_u_10_6"):
                    if hasattr(camera_frame, name):
                        u16 = self._as_ndarray(getattr(camera_frame, name), np.uint16)
                        if u16 is not None and u16.ndim == 2:
                            self._publish_raw_u16(u16)  # mono16 radiometry
                            self._publish_view_from_u16(u16)  # mono8 autoscale
                            return

            if self._mode == "float":
                for name in ("thermography_float", "thermometry_float"):
                    if hasattr(camera_frame, name):
                        f32 = self._as_ndarray(getattr(camera_frame, name), np.float32)
                        if f32 is not None and f32.ndim == 2:
                            self._publish_raw_f32(f32)  # 32FC1 radiometry
                            self._publish_view_from_f32(f32)  # mono8 autoscale
                            return

            if self._mode == "gray" and hasattr(camera_frame, "grayscale"):
                g = self._as_ndarray(camera_frame.grayscale, None)
                if g is not None and g.ndim == 2:
                    if g.dtype != np.uint16:
                        g = g.astype(np.uint16)
                    self._publish_raw_u16(g)
                    self._publish_view_from_u16(g)
                    return

            self.get_logger().warning(
                "Expected thermography plane not found; check installed SDK/binding plane names.",
                throttle_duration_sec=5.0,
            )
        except Exception as exc:
            self.get_logger().warning(f"Frame handling error: {exc}", throttle_duration_sec=2.0)

    @staticmethod
    def _as_ndarray(obj, dtype):
        if isinstance(obj, np.ndarray):
            arr = obj if dtype is None or obj.dtype == dtype else obj.astype(dtype)
            return np.ascontiguousarray(arr)
        data = getattr(obj, "data", None)
        h = getattr(obj, "height", getattr(obj, "rows", None))
        w = getattr(obj, "width", getattr(obj, "cols", None))
        if data is not None and h is not None and w is not None:
            dt = dtype if dtype is not None else np.uint16
            try:
                mv = memoryview(data)
                arr = np.frombuffer(mv, dtype=dt, count=int(h) * int(w)).reshape(int(h), int(w))
                return np.ascontiguousarray(arr)
            except Exception:
                return None
        try:
            arr = np.array(obj, copy=False)
            if dtype is not None and arr.dtype != dtype:
                arr = arr.astype(dtype)
            return np.ascontiguousarray(arr)
        except Exception:
            return None

    # Raw radiometry publishers
    def _publish_raw_u16(self, u16: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = int(u16.shape[0]), int(u16.shape[1])
        msg.encoding = "mono16"  # 16-bit single-channel grayscale (raw radiometry)
        msg.is_bigendian = 0
        msg.step = msg.width * 2
        msg.data = u16.tobytes()
        self.pub_raw.publish(msg)

    def _publish_raw_f32(self, f32: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = int(f32.shape[0]), int(f32.shape[1])
        msg.encoding = "32FC1"  # 32-bit float single channel (raw radiometry)
        msg.is_bigendian = 0
        msg.step = msg.width * 4
        msg.data = f32.tobytes()
        self.pub_raw.publish(msg)

    # Autoscaled preview (white-hot mono8)
    def _publish_view_from_u16(self, u16: np.ndarray):
        if self.pclip > 0.0:
            lo = np.percentile(u16, self.pclip)
            hi = np.percentile(u16, 100.0 - self.pclip)
        else:
            lo, hi = float(u16.min()), float(u16.max())
        if hi <= lo:
            hi = lo + 1.0
        norm = np.clip((u16.astype(np.float32) - lo) / (hi - lo), 0.0, 1.0)
        u8 = (norm * 255.0 + 0.5).astype(np.uint8)
        self._publish_mono8(u8)

    def _publish_view_from_f32(self, f32: np.ndarray):
        if self.pclip > 0.0:
            lo = np.percentile(f32, self.pclip)
            hi = np.percentile(f32, 100.0 - self.pclip)
        else:
            lo, hi = float(f32.min()), float(f32.max())
        if hi <= lo:
            hi = lo + 1e-3
        norm = np.clip((f32 - lo) / (hi - lo), 0.0, 1.0)
        u8 = (norm * 255.0 + 0.5).astype(np.uint8)
        self._publish_mono8(u8)

    def _publish_mono8(self, u8: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = int(u8.shape[0]), int(u8.shape[1])
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = msg.width
        msg.data = u8.tobytes()
        self.pub_view.publish(msg)


def main():
    rclpy.init()
    node = SeekThermalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

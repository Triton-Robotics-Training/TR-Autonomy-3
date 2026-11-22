#!/usr/bin/env python3
"""Publish the position error between HuskyBot's detection and the real panels.

Listens to the transforms broadcast by tf_broadcaster, finds which of the four
ground-truth panels (panel_0..panel_3) is closest to detected_panel, and
publishes detected - ground_truth for that panel on x_err, y_err and z_err.

Each detection is reported once. If there is no new detection, nothing is
published.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

DETECTED_PANEL_FRAME = 'detected_panel'
PANEL_FRAMES = [f'panel_{i}' for i in range(4)]


class CalcError(Node):
    def __init__(self):
        super().__init__('calc_error')

        # Frame the error is expressed in. 'map' gives world-frame x/y/z error;
        # try 'camera_frame' to see the error along the camera's own axes.
        self.declare_parameter('error_frame', 'map')

        self.error_frame = self.get_parameter('error_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x_pub = self.create_publisher(Float64, 'x_err', 10)
        self.y_pub = self.create_publisher(Float64, 'y_err', 10)
        self.z_pub = self.create_publisher(Float64, 'z_err', 10)

        self.last_stamp = None
        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        # Latest time at which detected_panel is available in the error frame.
        # That time is the detection's own timestamp, so every other lookup
        # below uses it too (same timestamp for everything, and never "now",
        # which would fail with an extrapolation-into-the-future error).
        try:
            detected = self.tf_buffer.lookup_transform(
                self.error_frame, DETECTED_PANEL_FRAME, Time())
        except TransformException:
            return  # tree isn't up yet, or nothing has been detected

        stamp = Time.from_msg(detected.header.stamp)

        # tf2 remembers the last detected_panel after detections stop, but its
        # timestamp stops changing, so an unchanged stamp means "no new
        # detection" and nothing is published.
        if self.last_stamp is not None and stamp == self.last_stamp:
            return

        d = detected.transform.translation
        best = None  # (distance, dx, dy, dz)
        for frame in PANEL_FRAMES:
            try:
                panel = self.tf_buffer.lookup_transform(
                    self.error_frame, frame, stamp)
            except TransformException as e:
                # usually extrapolation right at startup; retry on next tick
                self.get_logger().warn(
                    f'lookup of {frame} failed: {e}', throttle_duration_sec=2.0)
                return

            p = panel.transform.translation
            dx, dy, dz = d.x - p.x, d.y - p.y, d.z - p.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if best is None or dist < best[0]:
                best = (dist, dx, dy, dz)

        _, dx, dy, dz = best
        self.x_pub.publish(Float64(data=dx))
        self.y_pub.publish(Float64(data=dy))
        self.z_pub.publish(Float64(data=dz))
        self.last_stamp = stamp


def main():
    rclpy.init()
    node = CalcError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

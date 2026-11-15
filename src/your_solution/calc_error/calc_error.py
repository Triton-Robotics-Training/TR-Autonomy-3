import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

PANELS = ['panel_0', 'panel_1', 'panel_2', 'panel_3']


class CalcError(Node):

    def __init__(self):
        super().__init__('calc_error')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_stamp = None

        self.x_pub = self.create_publisher(Float64, 'x_err', 10)
        self.y_pub = self.create_publisher(Float64, 'y_err', 10)
        self.z_pub = self.create_publisher(Float64, 'z_err', 10)

        self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            latest = self.tf_buffer.lookup_transform('camera_frame', 'detected_panel', Time())
        except TransformException:
            return

        stamp = Time.from_msg(latest.header.stamp)
        if stamp.nanoseconds == self.last_stamp:
            return

        try:
            detected = self.tf_buffer.lookup_transform('map', 'detected_panel', stamp)
            truths = [self.tf_buffer.lookup_transform('map', p, stamp) for p in PANELS]
        except TransformException:
            return
        self.last_stamp = stamp.nanoseconds

        d = detected.transform.translation
        errors = []
        for truth in truths:
            p = truth.transform.translation
            errors.append((d.x - p.x, d.y - p.y, d.z - p.z))
        x_err, y_err, z_err = min(errors, key=lambda e: math.sqrt(e[0]**2 + e[1]**2 + e[2]**2))

        self.x_pub.publish(Float64(data=x_err))
        self.y_pub.publish(Float64(data=y_err))
        self.z_pub.publish(Float64(data=z_err))


def main(args=None):
    rclpy.init(args=args)
    node = CalcError()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener


class CalcError(Node):
    def __init__(self):
        super().__init__('calc_error')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x_pub = self.create_publisher(Float64, 'x_err', 10)
        self.y_pub = self.create_publisher(Float64, 'y_err', 10)
        self.z_pub = self.create_publisher(Float64, 'z_err', 10)

        self.last_stamp = None
        self.timer = self.create_timer(0.005, self.timer_callback)

    def timer_callback(self):
        try:
            detected = self.tf_buffer.lookup_transform('map', 'detected_panel', Time())
        except TransformException:
            return

        stamp = Time.from_msg(detected.header.stamp)
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return

        best_dist = float('inf')
        best_x = 0.0
        best_y = 0.0
        best_z = 0.0

        for i in range(4):
            try:
                panel = self.tf_buffer.lookup_transform('map', 'panel_' + str(i), stamp)
            except TransformException:
                return

            x_err = detected.transform.translation.x - panel.transform.translation.x
            y_err = detected.transform.translation.y - panel.transform.translation.y
            z_err = detected.transform.translation.z - panel.transform.translation.z
            dist = math.sqrt(x_err ** 2 + y_err ** 2 + z_err ** 2)

            if dist < best_dist:
                best_dist = dist
                best_x = x_err
                best_y = y_err
                best_z = z_err

        self.last_stamp = stamp

        x_msg = Float64()
        x_msg.data = best_x
        self.x_pub.publish(x_msg)

        y_msg = Float64()
        y_msg.data = best_y
        self.y_pub.publish(y_msg)

        z_msg = Float64()
        z_msg.data = best_z
        self.z_pub.publish(z_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CalcError()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

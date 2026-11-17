import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tr_messages.msg import SimGroundTruth
from std_msgs.msg import Float64

from turtlesim.srv import Spawn


class FrameListener(Node):

    def __init__(self):
        super().__init__('frame_listener')
        self.sub_truth = self.create_subscription(
            SimGroundTruth,
            'simulation/ground_truth',
            self.callback_truth,
            10)
        # Declare and acquire `target_frame` parameter
        self.detected_pane= self.declare_parameter(
        'target_frame', 'detected_panel').get_parameter_value().string_value
        self.goal= self.declare_parameter(
        'target_frame2', 'map').get_parameter_value().string_value
        self.camera= self.declare_parameter(
        'target_frame3', 'camera_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.px = self.create_publisher(Float64, 'x_err', 1)
        self.py = self.create_publisher(Float64, 'y_err', 1)
        self.pz = self.create_publisher(Float64, 'z_err', 1)

        # Call on_timer function every second
        #self.timer = self.create_timer(1.0, self.on_timer)

    def callback_truth (self, msg):
        if not self.detected_pane:
            self.get_logger().info('No detected_pane')
            return
        if not self.goal:
            self.get_logger().info('No map')
            return
        # if not self.tf_buffer.can_transform (self.camera, self.detected_pane, rclpy.time.Time()):
        #     self.get_logger().info('Can not be transformed')
        #     return
        self.get_logger().info('Was not returned')
        ok, reason = self.tf_buffer.can_transform(
            self.camera,
            self.detected_pane,
            rclpy.time.Time(),
            return_debug_tuple=True
            )
        if not ok:
            self.get_logger().warn(f'Cannot transform: {reason}')
            return
        t = self.tf_buffer.lookup_transform(
                    self.goal, #to
                    self.detected_pane,  #from
                    rclpy.time.Time())
        # t = self.tf_buffer.lookup_transform(
        #             self.goal, #to
        #             i,  #from
        #             rclpy.time.Time())
        x0 = msg.secondary_robot.armor_panel_poses[0].position.x
        y0 = msg.secondary_robot.armor_panel_poses[0].position.y
        z0 = msg.secondary_robot.armor_panel_poses[0].position.z

        x1 = msg.secondary_robot.armor_panel_poses[1].position.x
        y1 = msg.secondary_robot.armor_panel_poses[1].position.y
        z1 = msg.secondary_robot.armor_panel_poses[1].position.z

        x2 = msg.secondary_robot.armor_panel_poses[2].position.x
        y2 = msg.secondary_robot.armor_panel_poses[2].position.y
        z2 = msg.secondary_robot.armor_panel_poses[2].position.z

        x3 = msg.secondary_robot.armor_panel_poses[3].position.x
        y3 = msg.secondary_robot.armor_panel_poses[3].position.y
        z3 = msg.secondary_robot.armor_panel_poses[3].position.z

        x0error = abs (t.transform.translation.x - x0)/x0
        y0error = abs (t.transform.translation.y - y0)/y0
        z0error = abs (t.transform.translation.z - z0)/z0

        x1error = abs (t.transform.translation.x - x1)/x1
        y1error = abs (t.transform.translation.y - y1)/y1
        z1error = abs (t.transform.translation.z - z1)/z1

        x2error = abs (t.transform.translation.x - x2)/x2
        y2error = abs (t.transform.translation.y - y2)/y2
        z2error = abs (t.transform.translation.z - z2)/z2

        x3error = abs (t.transform.translation.x - x3)/x3
        y3error = abs (t.transform.translation.y - y3)/y3
        z3error = abs (t.transform.translation.z - z3)/z3

        error0 = x0error + y0error + z0error
        error1 = x1error + y1error + z1error
        error2 = x2error + y2error + z2error
        error3 = x3error + y3error + z3error

        outx = Float64 ()
        outy = Float64 ()
        outz = Float64 ()

        if error0 < error1 and error0 < error2 and error0 < error3:
            outx.data = float (x0error)
            outy.data = float (y0error)
            outz.data = float (z0error)
        elif error1 < error0 and error1 < error2 and error1 < error3:
            outx.data = float (x1error)
            outy.data = float (y1error)
            outz.data = float(z1error)
        elif error2 < error0 and error2 < error1 and error2 < error3:
            outx.data = float (x2error)
            outy.data = float (y2error)
            outz.data = float (z2error)
        else:
            outx.data = x3error
            outy.data = y3error
            outz.data = z3error
        
        self.px.publish(outx)
        self.py.publish(outy)
        self.pz.publish(outz)


    # def on_timer(self):
    #     # Store frame names in variables that will be used to
    #     # compute transformations

    #     if self.turtle_spawning_service_ready:
    #         if self.turtle_spawned:
    #             # Look up for the transformation between target_frame and turtle2 frames
    #             # and send velocity commands for turtle2 to reach target_frame
    #             try:
    #                 t = self.tf_buffer.lookup_transform(
    #                     'map', #to
    #                     self.detected_pane1,  #from
    #                     rclpy.time.Time())
    #             except TransformException as ex:
    #                 self.get_logger().info(
    #                     f'Could not transform')
    #                 return

    #             float abs (t.transform.translation.x - )

    #             msg = Twist()
    #             scale_rotation_rate = 1.0
    #             msg.angular.z = scale_rotation_rate * math.atan2(
    #                 t.transform.translation.y,
    #                 t.transform.translation.x)

    #             scale_forward_speed = 0.5
    #             msg.linear.x = scale_forward_speed * math.sqrt(
    #                 t.transform.translation.x ** 2 +
    #                 t.transform.translation.y ** 2)

    #             self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
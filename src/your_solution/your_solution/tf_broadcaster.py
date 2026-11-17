import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from tr_messages.msg import DetWithImg
from tr_messages.msg import SimGroundTruth

import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class TfBroadcaster(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')
        self.sub_detect = self.create_subscription(
            DetWithImg,
            'detections',
            self.callback_detect,
            10)
        self.sub_truth = self.create_subscription(
            SimGroundTruth,
            'simulation/ground_truth',
            self.callback_truth,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub_detect  # prevent unused variable warning
        self.sub_truth

    def callback_detect (self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        td = TransformStamped()
        td.header.stamp = self.get_clock().now().to_msg()
        td.header.frame_id = 'camera_frame'
        td.child_frame_id = 'detected_panel'
        td.transform.translation.x = msg.detection_info.detections[0].results[0].pose.pose.position.x
        td.transform.translation.y = msg.detection_info.detections[0].results[0].pose.pose.position.y
        td.transform.translation.z = msg.detection_info.detections[0].results[0].pose.pose.position.z

        td.transform.rotation.x = msg.detection_info.detections[0].results[0].pose.pose.orientation.x
        td.transform.rotation.y = msg.detection_info.detections[0].results[0].pose.pose.orientation.y
        td.transform.rotation.z = msg.detection_info.detections[0].results[0].pose.pose.orientation.z
        td.transform.rotation.w = msg.detection_info.detections[0].results[0].pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(td)


    def callback_truth (self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_frame'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.primary_robot.camera_pose.position.x
        t.transform.translation.y = msg.primary_robot.camera_pose.position.y
        t.transform.translation.z = msg.primary_robot.camera_pose.position.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        t.transform.rotation.x = msg.primary_robot.camera_pose.orientation.x
        t.transform.rotation.y = msg.primary_robot.camera_pose.orientation.y
        t.transform.rotation.z = msg.primary_robot.camera_pose.orientation.z
        t.transform.rotation.w = msg.primary_robot.camera_pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

        #00000000000000000000000000000000000000000000000000000000000000000000000

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'panel_0'
        t2.transform.translation.x = msg.secondary_robot.armor_panel_poses[0].position.x
        t2.transform.translation.y = msg.secondary_robot.armor_panel_poses[0].position.y
        t2.transform.translation.z = msg.secondary_robot.armor_panel_poses[0].position.z

        t2.transform.rotation.x = msg.secondary_robot.armor_panel_poses[0].orientation.x
        t2.transform.rotation.y = msg.secondary_robot.armor_panel_poses[0].orientation.y
        t2.transform.rotation.z = msg.secondary_robot.armor_panel_poses[0].orientation.z
        t2.transform.rotation.w = msg.secondary_robot.armor_panel_poses[0].orientation.w
        self.tf_broadcaster.sendTransform(t2)

        #00000000000000000000000000000000000000000000000000000000000000000000000

        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'map'
        t3.child_frame_id = 'panel_1'
        t3.transform.translation.x = msg.secondary_robot.armor_panel_poses[1].position.x
        t3.transform.translation.y = msg.secondary_robot.armor_panel_poses[1].position.y
        t3.transform.translation.z = msg.secondary_robot.armor_panel_poses[1].position.z

        t3.transform.rotation.x = msg.secondary_robot.armor_panel_poses[1].orientation.x
        t3.transform.rotation.y = msg.secondary_robot.armor_panel_poses[1].orientation.y
        t3.transform.rotation.z = msg.secondary_robot.armor_panel_poses[1].orientation.z
        t3.transform.rotation.w = msg.secondary_robot.armor_panel_poses[1].orientation.w
        self.tf_broadcaster.sendTransform(t3)

        #00000000000000000000000000000000000000000000000000000000000000000000000

        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = 'map'
        t4.child_frame_id = 'panel_2'
        t4.transform.translation.x = msg.secondary_robot.armor_panel_poses[2].position.x
        t4.transform.translation.y = msg.secondary_robot.armor_panel_poses[2].position.y
        t4.transform.translation.z = msg.secondary_robot.armor_panel_poses[2].position.z

        t4.transform.rotation.x = msg.secondary_robot.armor_panel_poses[2].orientation.x
        t4.transform.rotation.y = msg.secondary_robot.armor_panel_poses[2].orientation.y
        t4.transform.rotation.z = msg.secondary_robot.armor_panel_poses[2].orientation.z
        t4.transform.rotation.w = msg.secondary_robot.armor_panel_poses[2].orientation.w
        self.tf_broadcaster.sendTransform(t4)

        #00000000000000000000000000000000000000000000000000000000000000000000000

        t5 = TransformStamped()
        t5.header.stamp = self.get_clock().now().to_msg()
        t5.header.frame_id = 'map'
        t5.child_frame_id = 'panel_2'
        t5.transform.translation.x = msg.secondary_robot.armor_panel_poses[3].position.x
        t5.transform.translation.y = msg.secondary_robot.armor_panel_poses[3].position.y
        t5.transform.translation.z = msg.secondary_robot.armor_panel_poses[3].position.z

        t5.transform.rotation.x = msg.secondary_robot.armor_panel_poses[3].orientation.x
        t5.transform.rotation.y = msg.secondary_robot.armor_panel_poses[3].orientation.y
        t5.transform.rotation.z = msg.secondary_robot.armor_panel_poses[3].orientation.z
        t5.transform.rotation.w = msg.secondary_robot.armor_panel_poses[3].orientation.w
        self.tf_broadcaster.sendTransform(t5)

        #00000000000000000000000000000000000000000000000000000000000000000000000

        t5 = TransformStamped()
        t5.header.stamp = self.get_clock().now().to_msg()
        t5.header.frame_id = 'camera_frame'
        t5.child_frame_id = 'detected_panel'
        t5.transform.translation.x = msg.secondary_robot.armor_panel_poses[3].position.x
        t5.transform.translation.y = msg.secondary_robot.armor_panel_poses[3].position.y
        t5.transform.translation.z = msg.secondary_robot.armor_panel_poses[3].position.z

        t5.transform.rotation.x = msg.secondary_robot.armor_panel_poses[3].orientation.x
        t5.transform.rotation.y = msg.secondary_robot.armor_panel_poses[3].orientation.y
        t5.transform.rotation.z = msg.secondary_robot.armor_panel_poses[3].orientation.z
        t5.transform.rotation.w = msg.secondary_robot.armor_panel_poses[3].orientation.w
        self.tf_broadcaster.sendTransform(t5)







def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = TfBroadcaster()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
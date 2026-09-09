#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped, Transform

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header

from tf2_ros import TransformBroadcaster

from tr_messages.msg import SimGroundTruth

from geometry_msgs.msg import Pose, PoseArray

class FramePublisher(Node):
    def __init__(self):
        # Call constructor and give class a name
        super().__init__('tf_tree_helper')

        # Initialize tf2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create subscriptions
        self.subscription_true_pose = self.create_subscription(
            SimGroundTruth,
            f'/simulation/ground_truth',
            self.true_pose_handler, 
            10)

    def true_pose_handler(self, msg):

        self.create_pose_message('map', 'primary_camera_frame',msg.primary_robot.camera_pose, msg.header)
        self.create_pose_message('map', 'primary_lidar_frame',msg.primary_robot.lidar_pose, msg.header)
        self.create_pose_message('map', 'primary_turret_frame',msg.primary_robot.turret_pose, msg.header)
        self.create_pose_message('map', 'primary_chassis_frame',msg.primary_robot.chassis_pose, msg.header)

        self.create_pose_message('map', 'primary_panel_0', msg.primary_robot.armor_panel_poses[0], msg.header)
        self.create_pose_message('map', 'primary_panel_1', msg.primary_robot.armor_panel_poses[1], msg.header)
        self.create_pose_message('map', 'primary_panel_2', msg.primary_robot.armor_panel_poses[2], msg.header)
        self.create_pose_message('map', 'primary_panel_3', msg.primary_robot.armor_panel_poses[3], msg.header)

        self.create_pose_message('map', 'secondary_camera_frame',msg.secondary_robot.camera_pose, msg.header)
        self.create_pose_message('map', 'secondary_lidar_frame',msg.secondary_robot.lidar_pose, msg.header)
        self.create_pose_message('map', 'secondary_turret_frame',msg.secondary_robot.turret_pose, msg.header)
        self.create_pose_message('map', 'secondary_chassis_frame',msg.secondary_robot.chassis_pose, msg.header)

        self.create_pose_message('map', 'secondary_panel_0', msg.secondary_robot.armor_panel_poses[0], msg.header)
        self.create_pose_message('map', 'secondary_panel_1', msg.secondary_robot.armor_panel_poses[1], msg.header)
        self.create_pose_message('map', 'secondary_panel_2', msg.secondary_robot.armor_panel_poses[2], msg.header)
        self.create_pose_message('map', 'secondary_panel_3', msg.secondary_robot.armor_panel_poses[3], msg.header)

        self.create_pose_message('map', 'third_camera_frame',msg.third_robot.camera_pose, msg.header)
        self.create_pose_message('map', 'third_lidar_frame',msg.third_robot.lidar_pose, msg.header)
        self.create_pose_message('map', 'third_turret_frame',msg.third_robot.turret_pose, msg.header)
        self.create_pose_message('map', 'third_chassis_frame',msg.third_robot.chassis_pose, msg.header)

        self.create_pose_message('map', 'third_panel_0', msg.third_robot.armor_panel_poses[0], msg.header)
        self.create_pose_message('map', 'third_panel_1', msg.third_robot.armor_panel_poses[1], msg.header)
        self.create_pose_message('map', 'third_panel_2', msg.third_robot.armor_panel_poses[2], msg.header)
        self.create_pose_message('map', 'third_panel_3', msg.third_robot.armor_panel_poses[3], msg.header)

        self.create_pose_message('map', 'fourth_camera_frame',msg.fourth_robot.camera_pose, msg.header)
        self.create_pose_message('map', 'fourth_lidar_frame',msg.fourth_robot.lidar_pose, msg.header)
        self.create_pose_message('map', 'fourth_turret_frame',msg.fourth_robot.turret_pose, msg.header)
        self.create_pose_message('map', 'fourth_chassis_frame',msg.fourth_robot.chassis_pose, msg.header)

        self.create_pose_message('map', 'fourth_panel_0', msg.fourth_robot.armor_panel_poses[0], msg.header)
        self.create_pose_message('map', 'fourth_panel_1', msg.fourth_robot.armor_panel_poses[1], msg.header)
        self.create_pose_message('map', 'fourth_panel_2', msg.fourth_robot.armor_panel_poses[2], msg.header)
        self.create_pose_message('map', 'fourth_panel_3', msg.fourth_robot.armor_panel_poses[3], msg.header)

    # Utility function to create message so I don't have to do it manually
    def create_pose_message(self, parent_str: str, child_str: str, pose: Pose, header: Header):
        
        t = TransformStamped()

        t.header.frame_id = parent_str
        t.child_frame_id = child_str

        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation = pose.orientation

        t.header.stamp = header.stamp
        self.tf_broadcaster.sendTransform(t)

def main():
    print("running sim node tf helper")
    
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    print("shutting down sim node tf helper")
    node.destroy_node()
    rclpy.try_shutdown()
   
if __name__ == "__main__": 
    main()
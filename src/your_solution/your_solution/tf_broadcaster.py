import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from tr_messages.msg import SimGroundTruth

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class TFBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')

        self.detectSub = self.create_subscription(
            Pose, 
            'detections', 
            self.detectCallback, 
            10)

        self.groundSub = self.create_subscription(
            SimGroundTruth, 
            'simulation/ground_truth', 
            self.groundCallback, 
            10)

        self.tf_broadcaster = TransformBroadcaster(self)

    def detectCallback(self, msg):
        t = TransformStamped()

        xx = msg.data.position.x
        yy = msg.data.position.y
        zz = msg.data.position.z

        t.transform.translation.x = xx
        t.transform.translation.y = yy
        t.transform.translation.z = zz

        qData = msg.data.orientation
        q = [qData.x, qData.y, qData.z, qData.w]

        self.tf_broadcaster.sendTransform(t)

    def groundCallback(self, msg):

        # broadcasting the ground truth camera pose
        # PRIMARY robot
        t_prim = TransformStamped()

        cam_pose = msg.primary_robot.camera_pose.position
        cam_xx = cam_pose.x
        cam_yy = cam_pose.y
        cam_zz = cam_pose.z

        t_prim.transform.translation.x = cam_xx
        t_prim.transform.translation.y = cam_yy
        t_prim.transform.translation.z = cam_zz

        qData = msg.primary_robot.camera_pose.orientation
        q = [qData.x, qData.y, qData.z, qData.w]

        t_prim.transform.rotation.x = q[0]
        t_prim.transform.rotation.y = q[1]
        t_prim.transform.rotation.z = q[2]
        t_prim.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t_prim)

        #broadcasting the ground truth armor panel position
        # SECONDARY robot
        # is there a more efficient way to do this... T-T

        t_sec_pan1 = TransformStamped()
        t_sec_pan2 = TransformStamped()
        t_sec_pan3 = TransformStamped()

        pan1_pose = msg.secondary_robot.armor_panel_poses[0].position
        pan2_pose = msg.secondary_robot.armor_panel_poses[1].position
        pan3_pose = msg.secondary_robot.armor_panel_poses[2].position

        # pan_poses = [pan1_pose, pan2_pose, pan3_pose]

        pan1_xx = pan1_pose.x
        pan1_yy = pan1_pose.y
        pan1_zz = pan1_pose.z

        pan2_xx = pan2_pose.x
        pan2_yy = pan2_pose.y
        pan2_zz = pan2_pose.z

        pan3_xx = pan3_pose.x
        pan3_yy = pan3_pose.y
        pan3_zz = pan3_pose.z

        t_sec_pan1.transform.translation.x = pan1_xx
        t_sec_pan1.transform.translation.y = pan1_yy
        t_sec_pan1.transform.translation.z = pan1_zz

        t_sec_pan2.transform.translation.x = pan2_xx
        t_sec_pan2.transform.translation.y = pan2_yy
        t_sec_pan2.transform.translation.z = pan2_zz

        t_sec_pan3.transform.translation.x = pan3_xx
        t_sec_pan3.transform.translation.y = pan3_yy
        t_sec_pan3.transform.translation.z = pan3_zz

        qData_pan1 = msg.secondary_robot.armor_panel_poses[0].orientation
        qData_pan2 = msg.secondary_robot.armor_panel_poses[1].orientation
        qData_pan3 = msg.secondary_robot.armor_panel_poses[2].orientation
        
        q_pan1 = [qData_pan1.x, qData_pan1.y, qData_pan1.z, qData_pan1.w]
        q_pan2 = [qData_pan2.x, qData_pan2.y, qData_pan2.z, qData_pan2.w]
        q_pan3 = [qData_pan3.x, qData_pan3.y, qData_pan3.z, qData_pan3.w]

        t_sec_pan1.transform.rotation.x = q_pan1[0]
        t_sec_pan1.transform.rotation.y = q_pan1[1]
        t_sec_pan1.transform.rotation.z = q_pan1[2]
        t_sec_pan1.transform.rotation.w = q_pan1[3]

        t_sec_pan2.transform.rotation.x = q_pan2[0]
        t_sec_pan2.transform.rotation.y = q_pan2[1]
        t_sec_pan2.transform.rotation.z = q_pan2[2]
        t_sec_pan2.transform.rotation.w = q_pan2[3]

        t_sec_pan3.transform.rotation.x = q_pan3[0]
        t_sec_pan3.transform.rotation.y = q_pan3[1]
        t_sec_pan3.transform.rotation.z = q_pan3[2]
        t_sec_pan3.transform.rotation.w = q_pan3[3]

        self.tf_broadcaster.sendTransform(t_sec_pan1)
        self.tf_broadcaster.sendTransform(t_sec_pan2)
        self.tf_broadcaster.sendTransform(t_sec_pan3)

def main(args=None):
    rclpy.init(args=args)
    
    broadcaster = TFBroadcaster()
    
    rclpy.spin(broadcaster)

    tf_broadcaster.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
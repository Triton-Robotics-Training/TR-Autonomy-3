
import math

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tr_messages.msg import DetWithImg, SimGroundTruth

MAP_FRAME = 'map'
CAMERA_FRAME = 'camera_frame'
DETECTED_FRAME = 'detected_panel'


def pose_to_tf(pose, stamp, parent, child):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    q = pose.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-9:
        t.transform.rotation.w = 1.0
    else:
        t.transform.rotation.x = q.x / norm
        t.transform.rotation.y = q.y / norm
        t.transform.rotation.z = q.z / norm
        t.transform.rotation.w = q.w / norm
    return t


class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            SimGroundTruth, '/simulation/ground_truth', self.on_ground_truth, 10)
        self.create_subscription(
            DetWithImg, '/detections', self.on_detections, 10)

    def on_ground_truth(self, msg):
        stamp = msg.header.stamp
        tfs = [pose_to_tf(
            msg.primary_robot.camera_pose, stamp, MAP_FRAME, CAMERA_FRAME)]
        for i, panel in enumerate(msg.secondary_robot.armor_panel_poses):
            tfs.append(pose_to_tf(panel, stamp, MAP_FRAME, f'panel_{i}'))
        self.broadcaster.sendTransform(tfs)

    def on_detections(self, msg):
        detections = msg.detection_info.detections
        if not detections or not detections[0].results:
            return
        pose = detections[0].results[0].pose.pose
        stamp = msg.image.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = msg.detection_info.header.stamp
        self.broadcaster.sendTransform(
            pose_to_tf(pose, stamp, CAMERA_FRAME, DETECTED_FRAME))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TfBroadcaster())


if __name__ == '__main__':
    main()

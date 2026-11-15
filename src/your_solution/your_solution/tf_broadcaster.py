import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tr_messages.msg import DetWithImg, SimGroundTruth


def unit_quaternion(q):
    length = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if length < 1e-6:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return Quaternion(x=q.x / length, y=q.y / length, z=q.z / length, w=q.w / length)


def make_transform(parent, child, pose, stamp):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = unit_quaternion(pose.orientation)
    return t


class TfBroadcaster(Node):

    def __init__(self):
        super().__init__('tf_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.last_detection_stamp = None

        self.create_subscription(DetWithImg, '/detections', self.detections_callback, 10)
        self.create_subscription(
            SimGroundTruth, '/simulation/ground_truth', self.ground_truth_callback, 10)

    def ground_truth_callback(self, msg):
        stamp = msg.header.stamp
        transforms = [make_transform('map', 'camera_frame', msg.primary_robot.camera_pose, stamp)]
        for i, panel_pose in enumerate(msg.secondary_robot.armor_panel_poses):
            transforms.append(make_transform('map', f'panel_{i}', panel_pose, stamp))
        self.broadcaster.sendTransform(transforms)

    def detections_callback(self, msg):
        detections = msg.detection_info.detections
        if not detections or not detections[0].results:
            return

        stamp = msg.detection_info.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = msg.image.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        if (stamp.sec, stamp.nanosec) == self.last_detection_stamp:
            return
        self.last_detection_stamp = (stamp.sec, stamp.nanosec)

        pose = detections[0].results[0].pose.pose
        self.broadcaster.sendTransform(
            make_transform('camera_frame', 'detected_panel', pose, stamp))


def main(args=None):
    rclpy.init(args=args)
    node = TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
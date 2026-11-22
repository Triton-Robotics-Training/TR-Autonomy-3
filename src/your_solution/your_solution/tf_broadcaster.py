'''
Goal of this node is to publish the transforms/poses (poses can be thought of as transforms) as they update (because the panels and robots are moving) so that we can do error calculation

This node broadcasts (publish to /tf) the following so that Foxglove can overlay them and calculate error in part 2:

/detections - The pose of a detected panel relative to the camera frame (this makes it a "child" of camera)
simulation/ground_truth - The ground truth camera pose of our robot and the opponent robot's ground truth panel poses (panel_0, panel_1, pamnel_2, panel_3) in the world (map) frame

Part 2 then takes that data and calculates the error then displays everything with foxglove
'''

import rclpy #The ROS2 Python client library for using ROS2 in Python
from geometry_msgs.msg import Pose, TransformStamped #Pose is position + orientation so it's everything known about where the robot is and how it's oriented. TransformStamped is a transform which we will use to go from the camera frame to the map frame
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy #
from tf2_ros import TransformBroadcaster #publishes transforms as ROS topics
from tr_messages.msg import DetWithImg, SimGroundTruth #custom message defined at src/tr-utils/src/tr_messages/msg/simulation/SimGroundTruth.msg for the ground truth
 
MAP_FRAME = 'map'
CAMERA_FRAME = 'camera_frame'
DETECTED_PANEL_FRAME = 'detected_panel'
NUM_PANELS = 4
 
DETECTIONS_TOPIC = '/detections'
GROUND_TRUTH_TOPIC = '/simulation/ground_truth'
 
# HuskyBot publishes tr_messages/DetWithImg on /detections: a wrapper holding
# a vision_msgs/Detection2DArray (segmentation info i.e. bounding box) plus the camera image.
DETECTION_TYPES = {
    'tr_messages/msg/DetWithImg': DetWithImg,
}
 
 
class TfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster')
 
        self.tf_broadcaster = TransformBroadcaster(self)

        #Subscribe to the ground truths to compute transforms
        self.ground_truth_sub = self.create_subscription(
            SimGroundTruth, GROUND_TRUTH_TOPIC, self.ground_truth_callback, 10)
 
        #Subscribe to /detections as soon as a publisher for it shows up.
        self.detections_sub = None
        self.discovery_timer = self.create_timer(1.0, self.discover_detections)
 
    #Checks to see if detections have been published, subscribes to the publisher if they have
    def discover_detections(self):
        types = dict(self.get_topic_names_and_types()).get(DETECTIONS_TOPIC)
        if not types:
            return  # HuskyBot isn't up yet, try again next tick
 
        for type_name in types:
            msg_cls = DETECTION_TYPES.get(type_name)
            if msg_cls is not None:
                # best-effort so we match both reliable and best-effort publishers
                qos = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
                self.detections_sub = self.create_subscription(
                    msg_cls, DETECTIONS_TOPIC, self.detections_callback, qos)
                self.get_logger().info(
                    f'subscribed to {DETECTIONS_TOPIC} ({type_name})')
                self.discovery_timer.cancel()
                return
 
        self.get_logger().warn(
            f'{DETECTIONS_TOPIC} has type {types}, which is not in '
            f'DETECTION_TYPES; add it to tf_broadcaster.py',
            throttle_duration_sec=5.0)
 
    #Convert the pose of a child frame within the parent frame to a TransformStamped message that can be sent on /tf
    def make_transform(self, parent: str, child: str, pose: Pose, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        # Pose uses a Point + Quaternion, Transform uses a Vector3 + Quaternion,
        # so the position has to be copied field by field.
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        return t

    #Returns the first detection in a detection message that has a pose
    @staticmethod
    def first_detected_pose(msg):
        """Pose of the first detection that has one, else None."""
        for det in msg.detections:
            if det.results:
                return det.results[0].pose.pose
            # Detection3D also carries a pose on its bounding box
            center = getattr(getattr(det, 'bbox', None), 'center', None)
            if center is not None and hasattr(center, 'position'):
                return center
        return None
 
    #Format and broadcast ground truth poses as coming from the map frame
    def ground_truth_callback(self, msg: SimGroundTruth):
        stamp = msg.header.stamp
 
        transforms = [
            # camera pose of the PRIMARY robot, in the static map frame
            self.make_transform(
                MAP_FRAME, CAMERA_FRAME, msg.primary_robot.camera_pose, stamp)
        ]
 
        # armor panels of the SECONDARY robot, in the static map frame
        #Wraps each armor panel ground truth pose within the map frame
        panels = msg.secondary_robot.armor_panel_poses[:NUM_PANELS]
        for i, pose in enumerate(panels):
            transforms.append(
                self.make_transform(MAP_FRAME, f'panel_{i}', pose, stamp))
 
        self.tf_broadcaster.sendTransform(transforms)

    #Format and broadcast detections as coming from the camera frame
    def detections_callback(self, msg):
        # DetWithImg wraps the detections; the bare vision_msgs types don't
        array = getattr(msg, 'detection_info', msg)
 
        pose = self.first_detected_pose(array)
        if pose is None:
            return  # nothing detected this frame, so publish nothing
 
        stamp = array.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()
 
        # SolvePnP output is relative to the camera, so the parent is camera_frame
        self.tf_broadcaster.sendTransform(
            self.make_transform(CAMERA_FRAME, DETECTED_PANEL_FRAME, pose, stamp))
 
 
def main():
    rclpy.init()
    node = TfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
 
 
if __name__ == '__main__':
    main()

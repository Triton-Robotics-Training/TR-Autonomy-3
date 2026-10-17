import math

from std_msgs.msg import Float32

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class CalcError(Node):
    def __init__(self):
        super().__init__('calc_error')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publish_x_err = self.create_publisher(Float32, "x_err", 10)
        self.publish_y_err = self.create_publisher(Float32, "y_err", 10)
        self.publish_z_err = self.create_publisher(Float32, "z_err", 10)

        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        source_frame = 'map'
        child_frame_camPan = 'camera_frame'
        child_frame_pan1 = 'panel_1'
        child_frame_pan2 = 'panel_2'
        child_frame_pan3 = 'panel_3'
        child_frame_detect = 'detected_panel'
        msg_x = Float32()
        msg_y = Float32()
        msg_z = Float32()

        try:
            time = rclpy.time.Time()
            t_detect = self.tf_buffer.lookup_transform(
                source_frame,
                child_frame_detect,
                time
            )

            t_cam = self.tf_buffer.lookup_transform(
                source_frame,
                child_frame_camPan,
                time
            )

            t_pan1 = self.tf_buffer.lookup_transform(
                source_frame,
                child_frame_pan1,
                time
            )

            t_pan2 = self.tf_buffer.lookup_transform(
                source_frame,
                child_frame_pan2,
                time
            )

            t_pan3 = self.tf_buffer.lookup_transform(
                source_frame,
                child_frame_pan3,
                time
            )
            
            distToCam_x = t_detect.transform.translation.x - t_cam.transform.translation.x
            distToCam_y = t_detect.transform.translation.y - t_cam.transform.translation.y
            distToCam_z = t_detect.transform.translation.z - t_cam.transform.translation.z
            distToCam = math.sqrt(distToCam_x**2+distToCam_y**2+distToCam_z**2)

            distToPan1_x = t_detect.transform.translation.x - t_pan1.transform.translation.x
            distToPan1_y = t_detect.transform.translation.y - t_pan1.transform.translation.y
            distToPan1_z = t_detect.transform.translation.z - t_pan1.transform.translation.z
            distToPan1 = math.sqrt(distToPan1_x**2+distToPan1_y**2+distToPan1_z**2)

            distToPan2_x = t_detect.transform.translation.x - t_pan2.transform.translation.x
            distToPan2_y = t_detect.transform.translation.y - t_pan2.transform.translation.y
            distToPan2_z = t_detect.transform.translation.z - t_pan2.transform.translation.z
            distToPan2 = math.sqrt(distToPan2_x**2+distToPan2_y**2+distToPan2_z**2)

            distToPan3_x = t_detect.transform.translation.x - t_pan3.transform.translation.x
            distToPan3_y = t_detect.transform.translation.y - t_pan3.transform.translation.y
            distToPan3_z = t_detect.transform.translation.z - t_pan3.transform.translation.z
            distToPan3 = math.sqrt(distToPan3_x**2+distToPan3_y**2+distToPan3_z**2)

            distances = [distToCam, distToPan1, distToPan2, distToPan3]
            
            closest = 0
            for i in range(len(distances)):
                if (distances[i] < distances[closest]):
                    closest = i
            
            x_err = y_err = z_err = 0
            if (closest == 0):
                x_err = distToCam_x
                y_err = distToCam_y
                z_err = distToCam_z
            elif (closest == 1):
                x_err = distToPan1_x
                y_err = distToPan1_y
                z_err = distToPan1_z
            elif (closest == 2):
                x_err = distToPan2_x
                y_err = distToPan2_y
                z_err = distToPan2_z
            else:
                x_err = distToPan3_x
                y_err = distToPan3_y
                z_err = distToPan3_z
                
            msg_x.data = x_err
            msg_y.data = y_err
            msg_z.data = z_err

            self.publish_x_err.publish(msg_x)
            self.publish_y_err.publish(msg_y)
            self.publish_z_err.publish(msg_z)

        except TransformException as ex:
            self.get_logger().warning(
                f'Could not transform frames: {ex}'
            )
            return


def main(args=None):
    rclpy.init(args=args)
    
    calculator = CalcError()
    try:
        rclpy.spin(calculator)
    except KeyboardInterrupt:
        pass

    calculator.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
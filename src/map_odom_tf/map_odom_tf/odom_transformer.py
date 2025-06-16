#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
import tf_transformations
import math
from geometry_msgs.msg import Quaternion

class OdomTransformer(Node):
    def __init__(self):
        super().__init__('odom_transformer')

        # TF2 listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber and publisher
        self.sub = self.create_subscription(Odometry, '/odometry', self.odom_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odometry/map', 10)

    def odom_callback(self, msg):
        try:
            # Lookup the transform from map to odom
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,  # typically 'odom'
                rclpy.time.Time())  # latest available

            # === Transform position ===
            # Convert translation and rotation from transform
            t = transform.transform.translation
            r = transform.transform.rotation
            tf_matrix = tf_transformations.quaternion_matrix([r.x, r.y, r.z, r.w])
            tf_matrix[0][3] = t.x
            tf_matrix[1][3] = t.y
            tf_matrix[2][3] = t.z

            # Original position vector
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            pz = msg.pose.pose.position.z
            pos_in = [px, py, pz, 1.0]
            pos_out = tf_matrix @ pos_in  # Matrix multiply

            # === Transform orientation ===
            q1 = [r.x, r.y, r.z, r.w]  # map->odom
            q2 = [msg.pose.pose.orientation.x,
                  msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z,
                  msg.pose.pose.orientation.w]  # odom->base_link

            q_out = tf_transformations.quaternion_multiply(q1, q2)

            # === Construct new Odometry message ===
            new_msg = Odometry()
            new_msg.header = msg.header
            new_msg.header.frame_id = 'map'
            new_msg.child_frame_id = msg.child_frame_id
            new_msg.pose.pose.position.x = pos_out[0]
            new_msg.pose.pose.position.y = pos_out[1]
            new_msg.pose.pose.position.z = pos_out[2]
            new_msg.pose.pose.orientation = Quaternion(
                x=q_out[0], y=q_out[1], z=q_out[2], w=q_out[3]
            )
            new_msg.twist = msg.twist  # unchanged

            self.pub.publish(new_msg)

        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f'Transform lookup failed: {ex}')

def main():
    rclpy.init()
    node = OdomTransformer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

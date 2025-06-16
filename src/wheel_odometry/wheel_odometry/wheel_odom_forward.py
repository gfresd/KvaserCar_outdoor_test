import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32
import math

class RoverOdometryNode(Node):
    def __init__(self):
        super().__init__('rover_odometry_node')
        self.get_logger().info('RoverOdometryNode started!')

        # Declare parameters
        self.declare_parameter('wheelbase', 0.55)
        self.declare_parameter('update_frequency', 5.0)
        self.declare_parameter('is_radio', 1)

        # Read parameters
        self.wheelbase = self.get_parameter('wheelbase').value
        self.update_frequency = self.get_parameter('update_frequency').value
        self.is_radio = self.get_parameter('is_radio').value

        self.get_logger().info(f"Wheelbase: {self.wheelbase}, Update Frequency: {self.update_frequency} Hz, Controlled by radio: {self.is_radio}")

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.delta_radio = None
        self.delta_ros = None

        self.v_rear_left = 0.0
        self.v_rear_right = 0.0

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odometry', 10)

        # Subscribers
        self.create_subscription(Float32, '/rover/wheel_rear_left/speed_kph', self.rear_left_speed_callback, 10)
        self.create_subscription(Float32, '/rover/wheel_rear_right/speed_kph', self.rear_right_speed_callback, 10)
        self.create_subscription(Float32, '/rover/radio/steering', self.steering_radio_callback, 10)
        self.create_subscription(Float32, '/rover/steering', self.steering_ros_callback, 10)

        # Time tracking
        self.last_time = self.get_clock().now()

    def rear_left_speed_callback(self, msg):
        self.v_rear_left = msg.data / 3.6  # Convert kph to m/s

    def rear_right_speed_callback(self, msg):
        self.v_rear_right = msg.data / 3.6

    def steering_radio_callback(self, msg):
        self.delta_radio = msg.data / 360 * 2 * math.pi

    def steering_ros_callback(self, msg):
        self.delta_ros = msg.data / 360 * 2 * math.pi

    def compute_odometry(self):
        delta = self.delta_radio if self.is_radio else self.delta_ros
        if delta is None:
            self.get_logger().warn("Steering angle not received yet. Skipping this odometry update.", throttle_duration_sec=1.0)
            return

        current_time = self.get_clock().now()
        dt = 1 / self.update_frequency

        v_x = (self.v_rear_left + self.v_rear_right) / 2.0
        theta_dot = v_x / self.wheelbase * math.tan(delta)

        self.x += v_x * math.cos(self.theta) * dt
        self.y += v_x * math.sin(self.theta) * dt
        self.theta += theta_dot * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.angular.z = theta_dot

        odom_msg.pose.covariance = [float(x) for x in [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 10]]

        odom_msg.twist.covariance = [float(x) for x in [
            0.1, 0, 0, 0, 0, 0,
            0, 0.1, 0, 0, 0, 0,
            0, 0, 10, 0, 0, 0,
            0, 0, 0, 10, 0, 0,
            0, 0, 0, 0, 10, 0,
            0, 0, 0, 0, 0, 10]]

        self.odom_pub.publish(odom_msg)
        self.last_time = current_time

    def timer_callback(self):
        self.compute_odometry()

def main(args=None):
    rclpy.init(args=args)
    node = RoverOdometryNode()
    update_period = 1.0 / node.update_frequency
    node.create_timer(update_period, node.timer_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import socket
import json
import math
from collections import deque
import numpy as np
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, PolygonStamped, Point32
from std_msgs.msg import Float32  # for reference speed subscription

class Motion:
    """
    Class to simulate motion with bounded acceleration and compute displacement.

    Attributes
    ----------
    max_acc : float
        Maximum magnitude of acceleration / deceleration [m/s^2].
    delta_speed : float
        Threshold for considering target speed reached [m/s].
    v : float
        Current speed [m/s].
    t_total : float
        Total elapsed time [s].
    total_displacement : float
        Cumulative displacement [m].
    positions : List[float]
        Cumulative positions [m] at each timestep.
    speeds : List[float]
        Speeds [m/s] recorded at each step (before update).
    times : List[float]
        Time stamps [s].
    """
    def __init__(self, max_acc: float, delta_speed: float = 1e-1):
        self.max_acc = float(max_acc)
        self.delta_speed = float(delta_speed)
        self.v = 0.0
        self.t_total = 0.0
        self.total_displacement = 0.0
        self.positions: List[float] = [0.0]
        self.speeds: List[float] = [0.0]
        self.times: List[float] = [0.0]

    def reset(self) -> None:
        """Reset motion state to initial conditions."""
        self.v = 0.0
        self.t_total = 0.0
        self.total_displacement = 0.0
        self.positions = [0.0]
        self.speeds = [0.0]
        self.times = [0.0]

    def is_at_target(self, ref_speed: float) -> bool:
        """Check if current speed is within delta_speed of target."""
        return abs(self.v - ref_speed) <= self.delta_speed

    def update(self, ref_speed: float, dt: float) -> float:
        """
        Advance motion by dt seconds towards ref_speed with limited acceleration.

        Parameters
        ----------
        ref_speed : float
            Desired speed [m/s].
        dt : float
            Timestep duration [s].

        Returns
        -------
        disp : float
            Displacement during this timestep [m].
        """
        # Determine required acceleration
        dv = ref_speed - self.v
        if abs(dv) <= self.max_acc * dt or self.is_at_target(ref_speed):
            acc = dv / dt if dt > 0 else 0.0
        else:
            acc = math.copysign(self.max_acc, dv)

        # Update speed
        new_v = max(0.0, self.v + acc * dt)

        # Trapezoidal integration for displacement
        disp = 0.5 * (self.v + new_v) * dt

        # Update internal state
        self.t_total += dt
        self.total_displacement += disp
        self.positions.append(self.positions[-1] + disp)
        self.speeds.append(new_v)
        self.times.append(self.t_total)
        self.v = new_v

        return disp

    def get_total_displacement(self) -> float:
        """
        Return cumulative displacement since last reset.

        Returns
        -------
        total_displacement : float
            Total displacement [m].
        """
        return self.total_displacement

    def get_trajectory(self) -> np.ndarray:
        """
        Get arrays of time, speed, and position for the simulated trajectory.

        Returns
        -------
        traj : np.ndarray
            2D array with columns: time [s], speed [m/s], position [m].
        """
        return np.column_stack((np.array(self.times), np.array(self.speeds), np.array(self.positions)))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class EgoAdvSimNode(Node):
    def __init__(self):
        super().__init__('ego_adv_sim')
        # Declare parameters
        self.declare_parameter('ego_start_x', 4.0)
        self.declare_parameter('ego_start_y', 0.0)
        self.declare_parameter('ego_end_x', -4.0)
        self.declare_parameter('ego_end_y', 0.0)
        self.declare_parameter('ego_ref_speed', 1.5)
        self.declare_parameter('ego_max_acc', 1.2)

        self.declare_parameter('adv_start_x', 0.0)
        self.declare_parameter('adv_start_y', -4.0)
        self.declare_parameter('adv_end_x', 0.0)
        self.declare_parameter('adv_end_y', 5.0)
        self.declare_parameter('adv_ref_speed', 1.25)
        self.declare_parameter('adv_max_acc', 1.05)

        self.declare_parameter('cr_point_1', [0.5, 0.5])
        self.declare_parameter('cr_point_2', [0.5, -0.5])
        self.declare_parameter('cr_point_3', [-0.5, -0.5])
        self.declare_parameter('cr_point_4', [-0.5, 0.5])

        # UDP & sim parameters
        self.declare_parameter('udp_target_ip', '127.0.0.1')
        self.declare_parameter('udp_target_port', 9999)
        self.declare_parameter('udp_delay', 0.01) # in seconds
        self.declare_parameter('add_aoi', 850)     # in milli
        self.declare_parameter('adv_queue_size', 1)
        self.declare_parameter('comm_fail_start', 1.0)
        self.declare_parameter('comm_fail_duration', 9.0) # in seconds
        self.declare_parameter('sim_step', 0.02)
        self.declare_parameter('comm_step', 0.001)

        # vehcile sizes
        self.length = 0.720
        self.width = 0.515

        # Read parameters
        ego_start = (self.get_parameter('ego_start_x').value,
                     self.get_parameter('ego_start_y').value)
        ego_end = (self.get_parameter('ego_end_x').value,
                   self.get_parameter('ego_end_y').value)
        adv_start = (self.get_parameter('adv_start_x').value,
                     self.get_parameter('adv_start_y').value)
        adv_end = (self.get_parameter('adv_end_x').value,
                   self.get_parameter('adv_end_y').value)
        self.ego_ref_speed = self.get_parameter('ego_ref_speed').value
        ego_max_acc = self.get_parameter('ego_max_acc').value
        self.adv_ref_speed = self.get_parameter('adv_ref_speed').value
        adv_max_acc = self.get_parameter('adv_max_acc').value

        # Motion objects
        self.ego_motion = Motion(ego_max_acc)
        self.adv_motion = Motion(adv_max_acc)

        # Direction vectors and total distance
        def setup_motion(start, end, motion):
            dx, dy = end[0]-start[0], end[1]-start[1]
            dist = math.hypot(dx, dy)
            return (dx/dist, dy/dist), dist

        self.ego_dir, self.ego_path_dist = setup_motion(ego_start, ego_end, self.ego_motion)
        self.adv_dir, self.adv_path_dist = setup_motion(adv_start, adv_end, self.adv_motion)
        self.ego_start, self.ego_end = ego_start, ego_end
        self.adv_start, self.adv_end = adv_start, adv_end
        self.ego_yaw = math.atan2(self.ego_dir[1], self.ego_dir[0])
        self.adv_yaw = math.atan2(self.adv_dir[1], self.adv_dir[0])

        # QoS and publishers/subscribers
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                         depth=2,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        self.ego_pub = self.create_publisher(Odometry, '/odometry/map/sim', 1)
        self.adv_pub = self.create_publisher(Odometry, '/adv_emu_odometry', qos)
        self.ref_spd_sub = self.create_subscription(
            Float32, '/ref_spd', self._ref_spd_callback, qos)

        # UDP setup
        self.udp_ip = self.get_parameter('udp_target_ip').value
        self.udp_port = self.get_parameter('udp_target_port').value
        self._udp_delay_ns = int(self.get_parameter('udp_delay').value * 1e9)
        self.adv_queue_size = self.get_parameter('adv_queue_size').value
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.adv_queue = deque()
        self._msg_id = 0

        # Communication failure window
        self.start_ns = self.get_clock().now().nanoseconds
        cfs = self.get_parameter('comm_fail_start').value
        cfd = self.get_parameter('comm_fail_duration').value
        self._comm_fail_start = self.start_ns + int(cfs*1e9)
        self._comm_fail_end = self._comm_fail_start + int(cfd*1e9)
        self.add_aoi = self.get_parameter('add_aoi').value * 1e6

        # Path publishers for plotter
        self.coord_pub = {}
        for name, coord in [('ego_start', ego_start),
                            ('ego_end', ego_end),
                            ('adv_start', adv_start),
                            ('adv_end', adv_end)]:
            pub = self.create_publisher(Point, f'/{name}', qos)
            msg = Point(x=coord[0], y=coord[1], z=0.0)
            pub.publish(msg)

        self.crit_pub = self.create_publisher(
            PolygonStamped,
            '/critical_region',
            qos
        )

        # define your critical‐region polygon here (in map frame)
        region = PolygonStamped()
        region.header.frame_id = 'map'
        cr_point_1 = self.get_parameter('cr_point_1').value
        cr_point_2 = self.get_parameter('cr_point_2').value
        cr_point_3 = self.get_parameter('cr_point_3').value
        cr_point_4 = self.get_parameter('cr_point_4').value
        region.polygon.points = [
            Point32(x=cr_point_1[0], y=cr_point_1[1], z=0.0),
            Point32(x=cr_point_2[0], y=cr_point_2[1], z=0.0),
            Point32(x=cr_point_3[0], y=cr_point_3[1], z=0.0),
            Point32(x=cr_point_4[0], y=cr_point_4[1], z=0.0)
        ]

        # publish once (latched)
        self.crit_pub.publish(region)
        self.get_logger().info('Published critical region polygon')
        

        # Timers
        self.sim_timer = self.create_timer(self.get_parameter('sim_step').value,
                                           self._simulate)
        self.comm_timer = self.create_timer(self.get_parameter('comm_step').value,
                                            self._send_udp)

        self.get_logger().info('Simulation started')

    def _ref_spd_callback(self, msg: Float32):
        self.ego_ref_speed = msg.data

    def _simulate(self):
        # Check completion
        if (self.ego_motion.get_total_displacement() >= self.ego_path_dist or
            self.adv_motion.get_total_displacement() >= self.adv_path_dist):
            self.get_logger().info('Reached end of paths; stopping simulation')
            self.sim_timer.cancel()
            return

        # Ego
        _ = self.ego_motion.update(self.ego_ref_speed,
                                                  self.sim_timer.timer_period_ns/1e9)
        
        ego_total_displacement = self.ego_motion.get_total_displacement()
        
        ex = self.ego_start[0] + self.ego_dir[0] * ego_total_displacement
        ey = self.ego_start[1] + self.ego_dir[1] * ego_total_displacement
        ego_msg = Odometry()
        ego_msg.header.stamp = self.get_clock().now().to_msg()
        ego_msg.header.frame_id = 'map'
        ego_msg.child_frame_id = 'ego_base_link'
        ego_msg.pose.pose.position.x = ex
        ego_msg.pose.pose.position.y = ey
        ego_msg.pose.pose.orientation = quaternion_from_yaw(self.ego_yaw)
        ego_msg.twist.twist.linear.x = self.ego_motion.v
        self.ego_pub.publish(ego_msg)

        # Adv
        _ = self.adv_motion.update(self.adv_ref_speed,
                                                  self.sim_timer.timer_period_ns/1e9)
        
        adv_total_displacement = self.adv_motion.get_total_displacement()

        ax = self.adv_start[0] + self.adv_dir[0] * adv_total_displacement
        ay = self.adv_start[1] + self.adv_dir[1] * adv_total_displacement
        adv_msg = Odometry()
        adv_msg.header.stamp = self.get_clock().now().to_msg()
        adv_msg.header.frame_id = 'map'
        adv_msg.child_frame_id = 'adv_base_link'
        adv_msg.pose.pose.position.x = ax
        adv_msg.pose.pose.position.y = ay
        adv_msg.pose.pose.orientation = quaternion_from_yaw(self.adv_yaw)
        adv_msg.twist.twist.linear.x = self.adv_motion.v
        self.adv_pub.publish(adv_msg)

        # Queue UDP payload
        if len(self.adv_queue) < self.adv_queue_size:
            now = self.get_clock().now().nanoseconds
            noise = self._adv_noise()
            noise = 0
            send_v = self._add_vel_noise(self.adv_motion.v, noise)
            send_x, send_y = self._get_adv_position_to_send(ax, ay, noise)
            payload = {
                'id': self._msg_id,
                't_stamp': now,
                'front_x': send_x,
                'front_y': send_y,
                'vel': send_v
            }
            self.adv_queue.append(payload)
            self._msg_id += 1

    def _send_udp(self):
        now_ns = self.get_clock().now().nanoseconds
        # Stop both timers when both done
        if (self.ego_motion.get_total_displacement() >= self.ego_path_dist and
            self.adv_motion.get_total_displacement() >= self.adv_path_dist):
            self.get_logger().info('Paths done; stopping UDP sends')
            self.comm_timer.cancel()
            return

        # Suppress during comm failure
        if self._comm_fail_start <= now_ns < self._comm_fail_end:
            print(f"{(now_ns- self.start_ns)/1e9} COMM FAILURE")
            return

        # Send if delay elapsed
        if self.adv_queue and (now_ns - self.adv_queue[0]['t_stamp'] >= self._udp_delay_ns):
            info = self.adv_queue.popleft()
            info["t_stamp"] -= self.add_aoi 
            print(f"{(now_ns - self.start_ns)/1e9} SENDING {(info['t_stamp']- self.start_ns)/1e9} diff {(now_ns -info['t_stamp'])/1e9}")
            packet = json.dumps(info).encode('utf-8')
            self.udp_sock.sendto(packet, (self.udp_ip, self.udp_port))

    
    def _adv_noise(self) -> float:
        noise = np.random.normal(loc=0.0, scale=0.2, size=None)
        return noise

    def _add_vel_noise(self, value: float, noise:float) -> float:
        return value + noise
    
    def _get_adv_position_to_send(self, xc: float, yc:  float, noise: float):
        x = xc + self.width/2 
        y = yc + self.length/2
        
        x += noise
        y += noise

        return x, y


    def destroy_node(self):
        self.get_logger().info('Shutting down, closing UDP socket')
        self.udp_sock.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EgoAdvSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math
import json
import time
import threading
import socket

import tactical_node.critical_region as cr
from tactical_node.comm_msg import ComMsg
from tactical_node.logs import ExpLog
from tactical_node.tactical_behaviour import TacticalBehavior
import tactical_node.parameters as parameters
from tactical_node.ego_pose import EgoPose
from tactical_node.shared_object import SharedObj

from tactical_msgs.msg import LogEntry


# topics name
ROS_TOPIC_AEB = "/aeb_triggered"
ROS_TOPIC_ODOM = "/odometry/map/sim"
ROS_TOPIC_REF_VEL = "/ref_spd"
ROS_TOPIC_TACTICAL_LOG = "/tactical_log"
ROS_TOPIC_OBPS = "/obps"

ROS_TACTICAL_LOGIC_TIMER = 1/10

class TacticalNode(Node):

    def __init__(self):
        super().__init__('tactical_node')
        self.run = True
        self.pose = SharedObj()
        self.aeb = SharedObj()
        self.obps = SharedObj()

        self.sub_aeb = self.create_subscription(
            Bool,
            ROS_TOPIC_AEB,
            self.aeb_callback,
            1)

        self.sub_odom = self.create_subscription(
            Odometry,
            ROS_TOPIC_ODOM,
            self.odom_callback,
            1)
        
        qos_obps = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,         # only keep the last N messages
                depth=1,                                    # N = 1 (just the freshest)
                reliability=QoSReliabilityPolicy.RELIABLE,  # don’t retry old data
            )
        self.sub_obps_msg = self.create_subscription(
            String,
            ROS_TOPIC_OBPS,
            self.obps_callback,
            qos_obps)

        self.log_pub = self.create_publisher(LogEntry, ROS_TOPIC_TACTICAL_LOG, 10)

        self.logic_timer = self.create_timer(ROS_TACTICAL_LOGIC_TIMER, self.logic_callback)

        self.pub_vel = self.create_publisher(Float32, ROS_TOPIC_REF_VEL, 2)

        # experiment id
        self.start_id = -1
        self.start_time = -1

        # Declare parematers
        self.declare_parameter('cr_point_1',      parameters.CR_POINT_1)
        self.declare_parameter('cr_point_2',      parameters.CR_POINT_2)
        self.declare_parameter('cr_point_3',      parameters.CR_POINT_3)
        self.declare_parameter('cr_point_4',      parameters.CR_POINT_4)
        self.declare_parameter('adv_path_start',  parameters.ADV_PATH_START)
        self.declare_parameter('adv_path_end',    parameters.ADV_PATH_END)
        self.declare_parameter('ego_path_start',  parameters.EGO_PATH_START)
        self.declare_parameter('ego_path_end',    parameters.EGO_PATH_END)
        self.declare_parameter('ego_ref_speed',   parameters.EGO_REFERENCE_SPEED)
        self.declare_parameter('adv_max_speed',   parameters.ADV_MAX_SPEED)
        self.declare_parameter('ego_max_acc',     parameters.EGO_MAX_ACC)
        self.declare_parameter('ego_max_dec',     parameters.EGO_MAX_DEC)
        self.declare_parameter('adv_max_acc',     parameters.ADV_MAX_ACC)
        self.declare_parameter('ego_length',      parameters.EGO_LENGTH)
        self.declare_parameter('ego_width',       parameters.EGO_WIDTH)
        self.declare_parameter('adv_length',      parameters.ADV_LENGTH)
        self.declare_parameter('adv_width',       parameters.ADV_WIDTH)
        # Socket-start parameters
        self.declare_parameter('use_start_socket', False)
        self.declare_parameter('start_socket_host', '0.0.0.0')
        self.declare_parameter('start_socket_port', 12345)
        self.declare_parameter('bag_output_path', "")

        # read params them into member variables
        self.cr_point_1        = self.get_parameter('cr_point_1').value
        self.cr_point_2        = self.get_parameter('cr_point_2').value
        self.cr_point_3        = self.get_parameter('cr_point_3').value
        self.cr_point_4        = self.get_parameter('cr_point_4').value
        self.adv_path_start    = self.get_parameter('adv_path_start').value
        self.adv_path_end      = self.get_parameter('adv_path_end').value
        self.ego_path_start    = self.get_parameter('ego_path_start').value
        self.ego_path_end      = self.get_parameter('ego_path_end').value
        self.ego_ref_speed     = self.get_parameter('ego_ref_speed').value
        self.ego_max_acc       = self.get_parameter('ego_max_acc').value
        self.ego_max_dec       = self.get_parameter('ego_max_dec').value
        self.adv_max_acc       = self.get_parameter('adv_max_acc').value
        self.adv_max_speed     = self.get_parameter('adv_max_speed').value
        self.ego_length        = self.get_parameter('ego_length').value
        self.adv_length        = self.get_parameter('adv_length').value
        self.ego_width         = self.get_parameter('ego_width').value
        self.adv_width         = self.get_parameter('adv_width').value  
        self.use_start_socket  = self.get_parameter('use_start_socket').value  
        self.start_socket_host = self.get_parameter('start_socket_host').value
        self.start_socket_port = self.get_parameter('start_socket_port').value  
        ros_bag_path           = self.get_parameter('bag_output_path').value

        self.get_logger().info(f"REF SPEED FROM PARAMS= {self.ego_ref_speed}")

        self.ini_params = {
            "cr_point_1": self.cr_point_1,
            "cr_point_2": self.cr_point_2,
            "cr_point_3": self.cr_point_3,
            "cr_point_4": self.cr_point_4,
            "adv_path_start": self.adv_path_start,
            "adv_path_end": self.adv_path_end,
            "ego_path_start": self.ego_path_start,
            "ego_path_end": self.ego_path_end,
            "ego_ref_speed": self.ego_ref_speed,
            "adv_max_speed": self.adv_max_speed,
            "ego_max_acc": self.ego_max_acc,
            "ego_max_dec": self.ego_max_dec,
            "adv_max_acc": self.adv_max_acc,
            "ego_length": self.ego_length,
            "adv_length": self.adv_length,
            "ego_width": self.ego_width,
            "adv_width": self.adv_width,
            "use_start_socket": self.use_start_socket
        }

        # persistent logging
        self.exp_log = ExpLog(ros_bag_path)

        # critical region
        critical_region_poly = cr.create_cr_polygon([self.cr_point_1,
                                                     self.cr_point_2,
                                                     self.cr_point_3,
                                                     self.cr_point_4])

        target_path = cr.create_path(self.adv_path_start, self.adv_path_end)
        target_cr = cr.CriticalRegion(target_path, critical_region_poly)
        target_cr.compute_critical_points()

        ego_path = cr.create_path(self.ego_path_start, self.ego_path_end)
        ego_cr = cr.CriticalRegion(ego_path, critical_region_poly)
        ego_cr.compute_critical_points()

        # tactical behaviour
        self.behaviour = TacticalBehavior(
                                     ego_reference_speed=self.ego_ref_speed,
                                     ego_max_dec=self.ego_max_dec,
                                     ego_length=self.ego_length,
                                     target_max_acc=self.adv_max_acc,
                                     target_max_speed=self.adv_max_speed,
                                     target_length=self.adv_length,
                                     ego_critical_region=ego_cr,
                                     target_critical_region=target_cr)
        
        # If acting as server, start listening thread
        if self.use_start_socket:
            self.run = False
            threading.Thread(target=self._start_server, daemon=True).start()

    def aeb_callback(self, msg):
        # self.get_logger().info('AEB message is: "%s"' % msg.data)
        self.aeb.set_data(msg.data)

    def obps_callback(self, msg):
        #self.get_logger().info('OBPS message is: "%s"' % msg)
        content = json.loads(msg.data)

        com_msg = ComMsg( id=content["id"], 
                          time_stamp=content["t_stamp"],
                          arrival_time=time.time_ns(),
                          front=(content["front_x"], content["front_y"]), 
                          vel=content["vel"], 
                          length=self.adv_length)
        self.obps.set_data(com_msg)

    def odom_callback(self, msg):
        #self.get_logger().info('ODOM message is: "%s"' % msg.data)
        # Extract central point of the vehicle
        center_x = msg.pose.pose.position.x
        center_y = msg.pose.pose.position.y
        # Extract orientation quaternion
        orientation_q = msg.pose.pose.orientation
        r = R.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        # Convert to Euler angles: here 'xyz' = roll, pitch, yaw
        # `degrees=True` returns angles in degrees; omit for radians
        _, _, body_yaw = r.as_euler('xyz')
        # Calculate the front and rear point of the vehicle, considering the orientation
        front_x = center_x + self.ego_length/2*math.cos(body_yaw)
        front_y = center_y + self.ego_width/2*math.sin(body_yaw)
        rear_x = center_x - self.ego_length/2*math.cos(body_yaw)
        rear_y = center_y - self.ego_width/2*math.sin(body_yaw)
        # Extract vehicle velocity, vy is 0, since no lateral motion
        vel_x = msg.twist.twist.linear.x
        vel_y = 0
        ego_pose = EgoPose(front_x, front_y, rear_x, rear_y, vel_x, vel_y, self.ego_max_acc)
        self.pose.set_data(ego_pose)


    def publish_log(self):
        # build and publish a typed LogEntry
        entry = LogEntry()
        entry.header.stamp = self.get_clock().now().to_msg()

        entry.decision_time        = int(self.behaviour.decision_time)
        entry.aoi                  = int(self.behaviour.aoi)
        entry.aoi_in_sec           = float(self.behaviour.aoi_in_seconds)

        entry.ego_tactical_speed   = float(self.behaviour.ego_tactical_speed)
        entry.ego_front_d          = float(self.behaviour.ego_d_front)
        entry.ego_pos              = int(self.behaviour.ego_current_pos)
        entry.ego_pred_go_pos      = int(self.behaviour.ego_pred_go_pos)
        entry.ego_d_to_cr          = float(self.behaviour.ego_d_to_cr)
        entry.ego_ttcr             = float(self.behaviour.ego_ttcr)

        entry.target_acc           = float(self.behaviour.target_acc)
        entry.kalman_acc_val       = float(self.behaviour.kalman_acc_value)
        entry.target_ttcr          = float(self.behaviour.target_time_to_cr)
        entry.target_d_to_cr       = float(self.behaviour.target_d_to_cr)
        entry.target_pos           = int(self.behaviour.target_pred_pos)
        entry.target_d_front       = float(self.behaviour.target_prediction.d_front)

        if self.behaviour.target_front_coord_x is not None and self.behaviour.target_front_coord_y is not None:
            entry.target_front_coord_x = float(self.behaviour.target_front_coord_x)
            entry.target_front_coord_y = float(self.behaviour.target_front_coord_y)
        else:
            # just a large negative value
            entry.target_front_coord_x = float(-1_000)
            entry.target_front_coord_y = float(-1_000)

        self.log_pub.publish(entry)


    def logic_callback(self):
        cause, end = self.is_exp_ended()
        if end:
            self.get_logger().info("stopping the car, exp terminated with cause={0}".format(cause))
            self.pub_ref_speed(0.0)
            if self.run:
                self.exp_log.write_to_file(self.start_id, self.start_time, self.ini_params, self.behaviour.data_log, cause)
            self.run = False

        if self.run:
            msg = self.obps.get_data()
            ego_pose = self.pose.get_data()
            action = self.behaviour.decision(msg, ego_pose)
            speed = self.behaviour.action_to_speed(action)
            self.pub_ref_speed(speed)
            self.behaviour.log()
            self.publish_log()
        
        if self.run is False:
            self.pub_ref_speed(0.0)
    

    def is_exp_ended(self):
        aeb = self.aeb.get_data()            
        if aeb is not None:
            #self.get_logger().info("AEB IS {0}".format(aeb), throttle_duration_sec=1.0)
            if aeb is True:
                self.get_logger().warn("AEB IS TRUE", throttle_duration_sec=1.0)
                return "AEB", True
        
        self.get_logger().info("ego_front_d {0}".format(self.behaviour.ego_d_front), throttle_duration_sec=1.0)
        self.get_logger().info("path_length {0}".format(self.behaviour.ego_prediction.cr.cr_path.length), throttle_duration_sec=1.0)
        if self.behaviour.ego_d_front > self.behaviour.ego_prediction.cr.cr_path.length - 1:
            return "PASSED", True

        return None, False

    def pub_ref_speed(self, vel):
        self.get_logger().info(f"SENDING REF SPEED= {vel}")
        self.pub_vel.publish(Float32(data=vel))

    def _start_server(self):
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((self.start_socket_host, self.start_socket_port))
            server.listen(1)
            self.get_logger().info(f"Start server listening on {self.start_socket_host}:{self.start_socket_port}")
            
            conn, addr = server.accept()
            self.get_logger().info(f"Connection from {addr}, waiting for start signal")

            raw = conn.recv(1024)
            if not raw:
                self.get_logger().error("Connection closed before any data received")
                return

            try:
                payload = json.loads(raw.decode('utf-8').strip())
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to parse message: {e}")
                return

            cmd = payload.get("cmd")
            start_id = payload.get("start_id")
            if cmd != "start":
                self.get_logger().warning(f"Ignored cmd: {cmd!r}")
                return

            if not isinstance(start_id, str):
                self.get_logger().error(f"Invalid or missing 'start_id' field: {start_id!r}")
                self.get_logger().error(f"Received payload: {payload!r}")
                return

            self.get_logger().info(f"Received start command with start_id='{start_id}'")

            # Now kick off the run
            self.start_id = start_id
            self.start_time = time.time_ns()
            self.run = True

        finally:
            conn.close()
            server.close()


def main(args=None):
    rclpy.init(args=args)

    tactical_node = TacticalNode()

    try:
        rclpy.spin(tactical_node)
    except KeyboardInterrupt:
        tactical_node.exp_log.write_to_file(tactical_node.start_id, 
                                            tactical_node.start_time, 
                                            tactical_node.ini_params,
                                            tactical_node.behaviour.data_log, 
                                            "Force-exit")
    finally:
        tactical_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

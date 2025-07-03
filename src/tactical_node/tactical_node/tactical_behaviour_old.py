import time
import math
from enum import IntEnum

import shapely

from tactical_node.critical_region import CriticalRegion
from tactical_node.ego_pose import EgoPose
from tactical_node.ego_prediction import EgoPrediction
from tactical_node.kalman_filter import KalmanFilter
from tactical_node.target_prediction import TargetPrediction
from  tactical_node.comm_msg import ComMsg

def aoi_to_seconds(aoi) -> float:
    return aoi / 1_000_000_000

def get_time() -> int:
    return time.time_ns()


class TacticalAction(IntEnum):
    BREAKING = 0
    CONTINUE = 1


class TacticalBehavior:
    def __init__(self,
                 ego_reference_speed: float,
                 ego_max_dec: float,
                 ego_length: float,
                 target_max_acc: float,
                 target_length:float,
                 ego_critical_region: CriticalRegion,
                 target_critical_region: CriticalRegion):

        self.target_prediction = TargetPrediction(0, target_critical_region)
        self.adv_max_acc = target_max_acc
        self.adv_length = target_length
        self.ego_prediction = EgoPrediction(ego_reference_speed,
                                            ego_max_dec,
                                            ego_length,
                                            ego_critical_region)
        self.aoi = -1
        self.aoi_abs = -1
        self.aoi_in_seconds = -1
        self.msg = None
        self.ego_ttcr = -1
        self.target_ttcr = -1
        self.ego_d_to_cr = -1
        self.ego_d_front = -1
        self.target_front_coord_x, self.target_front_coord_y = None, None
        self.target_d_to_cr = -1
        self.ego_pos = CriticalRegion.Position.UNKNOWN
        self.ego_pred_go_pos = CriticalRegion.Position.UNKNOWN
        self.ego_pred_break_pos = CriticalRegion.Position.UNKNOWN
        self.target_pred_pos = CriticalRegion.Position.UNKNOWN
        self.reference_speed = ego_reference_speed
        self.ego_tactical_speed = self.reference_speed
        self.ego_action = TacticalAction.CONTINUE
        self.target_acc = -1
        self.kalman_acc_value = -1
        self.kalman_f = KalmanFilter(1)
        self.decision_time = -1
        self.call_time = -1
        self.data_log = {"call_time": list(),
                        "decision_time": list(),
                         "aoi": list(),
                         "aoi_abs":list(),
                         "aoi_seconds": list(),
                         "ego_tactical_speed": list(),
                         "ego_front_d": list(),
                         "ego_pos": list(),
                         "ego_pred_go_pos": list(),
                         "ego_d_to_cr": list(),
                         "ego_ttcr": list(),
                         "target_acc": list(),
                         "target_ttcr": list(),
                         "target_d_to_cr": list(),
                         "target_pos": list(),
                         "target_d_front": list(),
                         "target_front_coords": list(),
                         "target_vel" : list(),
                         "kalman_acc": list(),
                         "msg_current": list(),
                         "all_rec_msg": list(),              
                         }

    def _get_target_acc(self, aoi, velocity):

        self.kalman_acc_value = self.kalman_f.predict_acc(aoi, velocity)
        if self.kalman_acc_value > self.adv_max_acc:
            print("kalman acc is higher: {0}, assumption acc: {1}".format(self.kalman_acc_value, self.adv_max_acc))
            return self.kalman_acc_value

        return self.adv_max_acc
    
    def validate_new_msg(self, msg):
        if msg is not None:
            if len(self.data_log["all_rec_msg"]) > 0:
                _, temp_id, _ = self.data_log["all_rec_msg"][-1]
                if temp_id != msg.id:
                    # log it only if it is new
                    self.data_log["all_rec_msg"].append((self.call_time, msg.id, msg.time_stamp, msg.arrival_time))
            else:
                self.data_log["all_rec_msg"].append((self.call_time, msg.id, msg.time_stamp, msg.arrival_time))
            
            if self.msg is None:
                return msg
            
            if msg.time_stamp > self.msg.time_stamp:
                    return msg

        return self.msg


    def decision(self, msg: ComMsg, ego_pose: EgoPose) -> TacticalAction:
        """

        :param msg:
        :param ego_pose:
        :return:
        """
        self.call_time = get_time()
        self.ego_action = TacticalAction.BREAKING
        if ego_pose is None:
            return self.ego_action

        self.decision_time = get_time()
        self.ego_action = TacticalAction.CONTINUE
        ego_front_p = shapely.Point((ego_pose.front_x, ego_pose.front_y))
        shapely.prepare(ego_front_p)
        ego_vel = ego_pose.vel_fw
        ego_acc = ego_pose.acc_fw        
        
        self.msg = self.validate_new_msg(msg)
        self.ego_d_front, self.ego_d_to_cr, self.ego_ttcr = self.ego_prediction.get_dist_and_time_to_cr(ego_vel, ego_front_p)
        if self.msg is None:
            return self.ego_action
        
        self.aoi = self.decision_time - self.msg.time_stamp
        self.aoi_abs = math.fabs(self.aoi)
        self.aoi_in_seconds = aoi_to_seconds(self.aoi_abs) #transform in seconds!
        target_length = self.msg.length if self.msg.length is not None else self.adv_length
        target_front = shapely.Point(self.msg.front[0], self.msg.front[1])
        shapely.prepare(target_front)
        self.target_acc = self._get_target_acc(self.aoi_in_seconds, self.msg.velocity)
        self.target_pred_pos, self.target_ttcr = self.target_prediction.get_time_to_cr(
            self.aoi_in_seconds,
            target_front,
            target_length,
            self.msg.velocity,
            self.target_acc)

        self.ego_action = TacticalAction.CONTINUE

        if self.target_pred_pos == CriticalRegion.Position.BEFORE_CR:

            pred_go_pos, pred_break_ttcr = self.ego_prediction.get_predicted_positions(ego_vel,
                                                                                        ego_acc,
                                                                                        self.target_ttcr,
                                                                                        ego_front_p)
            if pred_go_pos == CriticalRegion.Position.AFTER_CR:
                self.ego_pos = pred_go_pos
                self.ego_action = TacticalAction.CONTINUE

            else:
                t_to_leave_cr = self.target_prediction.get_time_to_leave_cr(self.msg.velocity, self.target_acc)
                pred_go_leave_cr, _ = self.ego_prediction.get_predicted_positions(ego_vel,
                                                                                    ego_acc,
                                                                                    t_to_leave_cr,
                                                                                    ego_front_p)
                self.ego_pos = pred_go_leave_cr

                if pred_go_leave_cr == CriticalRegion.Position.INSIDE_CR:
                    self.ego_action = TacticalAction.BREAKING
                elif pred_go_leave_cr == CriticalRegion.Position.BEFORE_CR:
                    self.ego_action = TacticalAction.CONTINUE

        elif self.target_pred_pos == CriticalRegion.Position.AFTER_CR:
            self.ego_action = TacticalAction.CONTINUE

        elif self.target_pred_pos == CriticalRegion.Position.INSIDE_CR:
            _, _, self.ego_pos = self.ego_prediction.get_current_pos(ego_front_p)
            if self.ego_pos == CriticalRegion.Position.AFTER_CR:
                self.ego_action = TacticalAction.CONTINUE
            else:
                t_to_leave_cr = self.target_prediction.get_time_to_leave_cr(self.msg.velocity, self.target_acc)
                pred_go_leave_cr, _ = self.ego_prediction.get_predicted_positions(ego_vel,
                                                                                  ego_acc,
                                                                                  t_to_leave_cr,
                                                                                  ego_front_p)
                if pred_go_leave_cr != CriticalRegion.Position.BEFORE_CR:
                    self.ego_pos = pred_go_leave_cr
                    self.ego_action = TacticalAction.BREAKING

        self.ego_d_front, self.ego_d_to_cr, self.ego_ttcr = self.ego_prediction.get_dist_and_time_to_cr(ego_vel, ego_front_p)

        self.target_d_to_cr = self.target_prediction.dist_to_cr

        return self.ego_action

    def action_to_speed(self, action: TacticalAction):
        if action == TacticalAction.CONTINUE:
            self.ego_tactical_speed = self.reference_speed
        else:
            self.ego_tactical_speed = 0.0

        return self.ego_tactical_speed

    def log(self):
        self.data_log["call_time"].append(self.call_time)
        self.data_log["decision_time"].append(self.decision_time)
        self.data_log["ego_tactical_speed"].append(self.ego_tactical_speed)
        self.data_log["ego_front_d"].append(self.ego_d_front)
        self.data_log["ego_pos"].append(self.ego_pos)
        self.data_log["ego_pred_go_pos"].append(self.ego_pred_go_pos)
        self.data_log["ego_d_to_cr"].append(self.ego_d_to_cr)
        self.data_log["ego_ttcr"].append(self.ego_ttcr)
        self.data_log["aoi"].append(self.aoi)
        self.data_log["aoi_abs"].append(self.aoi_abs)
        self.data_log["aoi_seconds"].append(self.aoi_in_seconds)
       
        if self.msg is not None:
            self.data_log["msg_current"].append((self.call_time, self.msg.id, self.msg.time_stamp, msg.arrival_time))
            self.data_log["target_vel"].append(self.msg.velocity)

        self.data_log["target_acc"].append(self.target_acc)
        self.data_log["target_ttcr"].append(self.target_ttcr)
        self.data_log["target_d_to_cr"].append(self.target_d_to_cr)
        self.data_log["target_pos"].append(self.target_pred_pos)
        self.data_log["target_d_front"].append(self.target_prediction.d_front)

       
        
        if self.target_prediction.d_front != -1:
            self.target_front_coord_x, self.target_front_coord_y = self.target_prediction.get_coords_of_projected_front()
            self.data_log["target_front_coords"].append((self.target_front_coord_x, self.target_front_coord_y))

        self.data_log["kalman_acc"].append(self.kalman_acc_value)

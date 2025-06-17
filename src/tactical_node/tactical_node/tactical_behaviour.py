import time
import math
from enum import IntEnum

import shapely

from tactical_node.critical_region import CriticalRegion
from tactical_node.ego_pose import EgoPose
from tactical_node.ego_prediction import EgoPrediction
from tactical_node.kalman_filter import KalmanFilter
from tactical_node.target_prediction import TargetPrediction
from tactical_node.comm_msg import ComMsg

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
                 target_max_speed: float,
                 target_length:float,
                 ego_critical_region: CriticalRegion,
                 target_critical_region: CriticalRegion):

        self.target_prediction = TargetPrediction(0, target_critical_region, target_max_speed)
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
        self.target_time_to_cr = -1
        self.target_time_to_leave_cr = -1
        self.ego_d_to_cr = -1
        self.ego_d_front = -1
        self.ego_time_to_leave_cr = -1
        self.ego_time_to_cr = -1
        #-- logging
        self.ego_ttlcr_less_adv_ttcr = -1
        self.ego_ttcr_greater_adv_ttlcr = -1
        self.ego_ttcr_less_adv_ttlcr = -1
        # ---
        self.target_front_coord_x, self.target_front_coord_y = None, None
        self.target_d_to_cr = -1
        self.ego_current_pos = CriticalRegion.Position.UNKNOWN
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
        self.data_log = {"version": 2,
                         "call_time": list(),
                         "decision_time": list(),
                         "aoi": list(),
                         "aoi_abs":list(),
                         "aoi_seconds": list(),
                         "ego_tactical_speed": list(),
                         "ego_front_d": list(),
                         "ego_current_pos": list(),
                         "ego_pred_go_pos": list(),
                         "ego_d_to_cr": list(),
                         "ego_ttcr": list(),
                         "ego_time_to_leave_cr": list(),
                         "ego_time_to_cr": list(),
                         "ego_ttlcr_less_adv_ttcr": list(),
                         "ego_ttcr_greater_adv_ttlcr": list(),
                         "ego_ttcr_less_adv_ttlcr" :list(),
                         "target_acc": list(),
                         "target_ttcr": list(),
                         "target_time_to_leave_cr": list(),
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
                _, temp_id, _, _ = self.data_log["all_rec_msg"][-1]
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
    
    def pose_to_string(self, pose):
        if pose == CriticalRegion.Position.UNKNOWN:
            return "UNKNOWN"
        
        if pose == CriticalRegion.Position.BEFORE_CR:
            return "BEFORE_CR"
        
        if pose == CriticalRegion.Position.INSIDE_CR:
            return "INSIDE_CR"
        
        if pose == CriticalRegion.Position.AFTER_CR:
            return "AFTER_CR"


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
        
        #---- AGE of INFORMATION
        self.aoi = self.decision_time - self.msg.time_stamp
        self.aoi_abs = math.fabs(self.aoi)
        # transform AoI in seconds!
        self.aoi_in_seconds = aoi_to_seconds(self.aoi_abs) 

        #--- RESET VARIABLES
        #reset time to cr variables
        self.ego_time_to_leave_cr = -1
        self.ego_time_to_cr = -1
        self.target_time_to_leave_cr = -1

        #logging
        self.ego_ttlcr_less_adv_ttcr = -1
        self.ego_ttcr_greater_adv_ttlcr = -1
        self.ego_ttcr_less_adv_ttlcr = -1

        #reset ego position variable
        self.ego_pred_go_pos = CriticalRegion.Position.UNKNOWN

        #---- COMPUTE TARGET POSITION AND TIME TO CR
        target_length = self.msg.length if self.msg.length is not None else self.adv_length
        target_front = shapely.Point(self.msg.front[0], self.msg.front[1])
        shapely.prepare(target_front)
        self.target_acc = self._get_target_acc(self.aoi_in_seconds, self.msg.velocity)
        self.target_pred_pos, self.target_time_to_cr = self.target_prediction.estimate_time_to_cr(
            self.aoi_in_seconds,
            target_front,
            target_length,
            self.msg.velocity,
            self.target_acc)

        #---- COMPUTE CASES
        # get ego current position
        _, _, self.ego_current_pos = self.ego_prediction.get_current_pos(ego_front_p)
        self.ego_action = TacticalAction.CONTINUE
        #print("**********INITIAL, position: {0}".format(self.pose_to_string(self.target_pred_pos)) )

        #--- CASE target after the CR
        if self.target_pred_pos == CriticalRegion.Position.AFTER_CR:
            #print("**********AFTER!!!")
            self.ego_action = TacticalAction.CONTINUE

        #--- CASE target before the CR
        elif self.target_pred_pos == CriticalRegion.Position.BEFORE_CR:
                                                                                    
            # get the time the target will take to be AFTER the CR
            self.target_time_to_leave_cr = self.target_prediction.get_time_to_leave_cr(self.target_pred_pos, self.msg.velocity, self.target_acc)

            
            #predict ego time to CR and to leave CR
            self.ego_time_to_leave_cr = self.ego_prediction.get_time_to_leave_cr(self.ego_current_pos,
                                                                          ego_vel,
                                                                          ego_acc)
            
            self.ego_time_to_cr = self.ego_prediction.get_time_to_cr(self.ego_current_pos,
                                                                ego_vel,
                                                                ego_acc)

            # check where ego is
            if self.ego_current_pos == CriticalRegion.Position.BEFORE_CR:
                #print("***********PPPPPPPPPPPPPPP!!!!!! pred_pos_1: {0} pred_pos_2: {1}".format(self.pose_to_string(ego_pred_go_pos_ttcr1), self.pose_to_string(ego_pred_go_pos_ttcr2)))
                if self.ego_time_to_leave_cr < self.target_time_to_cr:
                    self.ego_action = TacticalAction.CONTINUE
                    self.ego_pred_go_pos = CriticalRegion.Position.AFTER_CR
                    self.ego_ttlcr_less_adv_ttcr = 1 
                    #print(f"***********HERE 11  ego_time_to_leave_cr {ego_time_to_leave_cr}!!!!!!")
                elif self.ego_time_to_cr > self.target_time_to_leave_cr:
                    self.ego_action = TacticalAction.CONTINUE
                    self.ego_pred_go_pos = CriticalRegion.Position.BEFORE_CR
                    self.ego_ttcr_greater_adv_ttlcr = 1
                    #print(f"***********HERE 22 ego_time_to_cr {ego_time_to_cr}!!!!!!")
                else:
                    self.ego_action = TacticalAction.BREAKING
                    self.ego_pred_go_pos = CriticalRegion.Position.INSIDE_CR
                    self.ego_ttlcr_less_adv_ttcr = 0
                    self.ego_ttcr_greater_adv_ttlcr = 0
                    #print("***********HERE 33!!!!!!")
                    
            elif self.ego_current_pos == CriticalRegion.Position.INSIDE_CR:
                # in case ego is inside or after the CR
                # the timing were checked
                self.ego_action = TacticalAction.CONTINUE
                #print("***********OOOOOOOOOOOOOOOO!!!!!!")
            else:
                # ego is AFTER the CR
                self.ego_action = TacticalAction.CONTINUE  
                #print("***********TTTTTTTTTTTTTTT!!!!!!")    

        #--- CASE target inside the CR
        elif self.target_pred_pos == CriticalRegion.Position.INSIDE_CR:
            
            # get the time the target needs to move out from the CR
            self.target_time_to_leave_cr = self.target_prediction.get_time_to_leave_cr(self.target_pred_pos, self.msg.velocity, self.target_acc)

            if self.target_time_to_leave_cr == TargetPrediction.NO_TIME_TO_CR:
                # this should not be possible because target is INSIDE_CR
                print("[ERROR] (1) should not be possible!")
                self.ego_action = TacticalAction.CONTINUE
                #print("***********AAAAAAA!!!!!!")
            else: 
                # get the predicted ego position using the target time to leave the CR
                self.ego_pred_go_pos, _ = self.ego_prediction.get_predicted_positions(ego_vel,
                                                                                      ego_acc,
                                                                                      self.target_time_to_leave_cr,
                                                                                      ego_front_p)
                
                if self.ego_current_pos == CriticalRegion.Position.AFTER_CR:
                    self.ego_action = TacticalAction.CONTINUE
                    #print("***********BBBBBBBBBB!!!!!!")
                
                elif self.ego_current_pos == CriticalRegion.Position.INSIDE_CR:
                    # could result in anavoidable crash (depending on dynamics and CR dimension)
                    self.ego_action = TacticalAction.BREAKING
                    #print("***********CCCCCCCC!!!!!!")

                elif self.ego_current_pos == CriticalRegion.Position.BEFORE_CR:
                    self.ego_time_to_cr = self.ego_prediction.get_time_to_cr(self.ego_current_pos,
                                                                ego_vel,
                                                                ego_acc)
                    
                    if self.ego_time_to_cr < self.target_time_to_leave_cr:
                        # ego must break
                        self.ego_action = TacticalAction.BREAKING
                        self.ego_ttcr_less_adv_ttlcr = 1
                    else:
                        # ego can continue
                        self.ego_action = TacticalAction.CONTINUE
                        self.ego_ttcr_less_adv_ttlcr = 0

                else:
                    # ego current pose is before the CR                                    
                    print("[ERROR] we should not be here")

        #print("*************OUT")
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
        self.data_log["ego_current_pos"].append(self.ego_current_pos)
        self.data_log["ego_pred_go_pos"].append(self.ego_pred_go_pos)
        self.data_log["ego_d_to_cr"].append(self.ego_d_to_cr)
        self.data_log["ego_ttcr"].append(self.ego_ttcr)
        self.data_log["ego_time_to_cr"].append(self.ego_time_to_cr)
        self.data_log["ego_time_to_leave_cr"].append(self.ego_time_to_leave_cr)
        self.data_log["aoi"].append(self.aoi)
        self.data_log["aoi_abs"].append(self.aoi_abs)
        self.data_log["aoi_seconds"].append(self.aoi_in_seconds)

        self.data_log["ego_ttlcr_less_adv_ttcr"].append(self.ego_ttlcr_less_adv_ttcr)
        self.data_log["ego_ttcr_greater_adv_ttlcr"].append(self.ego_ttcr_greater_adv_ttlcr)
        self.data_log["ego_ttcr_less_adv_ttlcr"].apend(self.ego_ttcr_less_adv_ttlcr)
       
        if self.msg is not None:
            self.data_log["msg_current"].append((self.call_time, self.msg.id, self.msg.time_stamp, self.msg.arrival_time))
            self.data_log["target_vel"].append(self.msg.velocity)

        self.data_log["target_acc"].append(self.target_acc)
        self.data_log["target_ttcr"].append(self.target_time_to_cr)
        self.data_log["target_time_to_leave_cr"].append(self.target_time_to_leave_cr)
        self.data_log["target_d_to_cr"].append(self.target_d_to_cr)
        self.data_log["target_pos"].append(self.target_pred_pos)
        self.data_log["target_d_front"].append(self.target_prediction.d_front)

       
        
        if self.target_prediction.d_front != -1:
            self.target_front_coord_x, self.target_front_coord_y = self.target_prediction.get_coords_of_projected_front()
            self.data_log["target_front_coords"].append((self.target_front_coord_x, self.target_front_coord_y))

        self.data_log["kalman_acc"].append(self.kalman_acc_value)

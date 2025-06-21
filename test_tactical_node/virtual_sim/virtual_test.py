import time
import shapely
import math

import parameters as parameters
from logs_test import TestExpLog

import critical_region as critical_region
from tactical_behaviour import TacticalBehavior
from ego_pose import EgoPose
from comm_msg import ComMsg

from motion import Motion

from trajectory_plotter import TrajectoryPlotter

def is_ended(b: TacticalBehavior, front_ego, front_target):
    if b.ego_d_front > b.ego_prediction.cr.cr_path.length - 1:
        return "PASSED", True

    if shapely.distance(front_target, front_ego) < 0.1:
        return "CRASH", True

    return None, False


def pub_ego_ref_speed(vel):
    print("pub speed: {0}".format(vel))



class OBPS:
    def __init__(self, delay=60):
        self.delay = delay * 1e6
        self.msg = None
        self.count = 0
        self.msg_id = 0

    def get_msg(self, adv_v, front_target_x, front_target_y):
        self.msg = self._make_msg(adv_v, front_target_x, front_target_y)

        if self.count == 0:
            self.count = 1
            return None
        else:
            self.msg.time_stamp += self.delay
            return self.msg

    
    def get_static_msg(self):
        return ComMsg(0,
                time.time_ns(),
                (0.5 , 0),
                (0.5, 0 - parameters.ADV_LENGTH),
                0)
    
    def _get_noise(self) -> float:
        noise = -0.2 
        return 0.0


    def _make_msg(self, adv_v, front_target_x, front_target_y):
        out_front_y = front_target_y + self._get_noise()
        adv_v = adv_v + self._get_noise()
        self.msg_id += 1

        return ComMsg(self.msg_id,
                time.time_ns(),
                (front_target_x , out_front_y),
                (front_target_x, out_front_y - parameters.ADV_LENGTH),
                adv_v)




if __name__ == '__main__':

    critical_region_poly = critical_region.create_cr_polygon([parameters.CR_POINT_1,
                                                              parameters.CR_POINT_2,
                                                              parameters.CR_POINT_3,
                                                              parameters.CR_POINT_4])

    target_path = critical_region.create_path(parameters.ADV_PATH_START, parameters.ADV_PATH_END)
    target_cr = critical_region.CriticalRegion(target_path, critical_region_poly)
    target_cr.compute_critical_points()

    ego_path = critical_region.create_path(parameters.EGO_PATH_START, parameters.EGO_PATH_END)
    ego_cr = critical_region.CriticalRegion(ego_path, critical_region_poly)
    ego_cr.compute_critical_points()

    #target_cr.plot_regions("TARGET")
    #ego_cr.plot_regions("EGO")

    behaviour = TacticalBehavior(ego_reference_speed=parameters.EGO_REFERENCE_SPEED,
                                 ego_max_dec=parameters.EGO_MAX_DEC,
                                 ego_length=parameters.EGO_LENGTH,
                                 target_max_acc=parameters.ADV_MAX_ACC,
                                 target_max_speed=parameters.ADV_MAX_SPEED,
                                 target_length=parameters.ADV_LENGTH,
                                 ego_critical_region=ego_cr,
                                 target_critical_region=target_cr)

    ego_motion = Motion(max_acc=parameters.EGO_MAX_ACC)
    adv_motion = Motion(max_acc=parameters.ADV_MAX_ACC)

    end = False
    cause = "Unknown"
    ego_ref = parameters.EGO_REFERENCE_SPEED
    log_debug = {"target_front_d":list(), "ego_front_d":list(), "ego_v":list(), "target_v":list()}
    DELTA_T = 0.1

    #---- PLOTTING
    plotter = TrajectoryPlotter()
    plotter.set_critical_region([parameters.CR_POINT_1, parameters.CR_POINT_2, parameters.CR_POINT_3, parameters.CR_POINT_4])
    plotter.set_ego_path(parameters.EGO_PATH_START, parameters.EGO_PATH_END)
    plotter.set_adv_path(parameters.ADV_PATH_START, parameters.ADV_PATH_END)

    #---- COMMUNICATION
    obps = OBPS(1000)

    while 1:
        time.sleep(DELTA_T)
        if end:
            print("stopping the car, exp terminated with cause={0}".format(cause))
            pub_ego_ref_speed(0.0)
            exp_log = TestExpLog(behaviour.data_log, log_debug, cause)
            exp_log.write_to_file()
            break

        _ = adv_motion.update(parameters.ADV_MAX_SPEED, DELTA_T)
        total_dist_target = adv_motion.get_total_displacement()
        adv_center = target_path.interpolate(total_dist_target)
        front_target = target_path.interpolate(total_dist_target + parameters.ADV_LENGTH/2)
        log_debug["target_front_d"].append(total_dist_target)
        log_debug["target_v"].append(adv_motion.v)

        _ = ego_motion.update(ego_ref, DELTA_T)
        total_dist_ego = ego_motion.get_total_displacement()
        ego_center = ego_path.interpolate(total_dist_ego)
        front_ego = ego_path.interpolate(total_dist_ego + parameters.EGO_LENGTH/2)
        log_debug["ego_front_d"].append(total_dist_ego)
        log_debug["ego_v"].append(ego_motion.v)

        msg = obps.get_msg(adv_motion.v, front_target.x, front_target.y)
        #msg = get_obps_static_msg()
        #msg = None
        ego_pose = EgoPose(front_x=front_ego.x,
                           front_y=front_ego.y,
                           rear_x=front_ego.x - parameters.EGO_LENGTH,
                           rear_y=front_ego.y - parameters.EGO_LENGTH,
                           vel_x=ego_motion.v,
                           vel_y=0,
                           ego_max_acc=parameters.EGO_MAX_ACC)

        action = behaviour.decision(msg, ego_pose)
        ego_ref = behaviour.action_to_speed(action)
        behaviour.log()
        cause, end = is_ended(behaviour, front_ego, front_target)
        print("ego_d_front ", behaviour.ego_d_front)

        plotter.update_ego_pose(ego_center.x, ego_center.y, 0)
        plotter.update_adv_pose(adv_center.x, adv_center.y, -math.pi/2)

        if behaviour.target_prediction.d_front != -1:
            target_front_coord_x, target_front_coord_y = behaviour.target_prediction.get_coords_of_projected_front()
            plotter.set_tactical_front(target_front_coord_x, target_front_coord_y)
        
        
        plotter.render()

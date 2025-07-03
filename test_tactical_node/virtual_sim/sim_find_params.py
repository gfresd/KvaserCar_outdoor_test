import time
import queue
import shapely
from tqdm import tqdm
import itertools
import json
import multiprocessing
import numpy as np

import parameters as parameters
from logs_test import TestExpLog
import critical_region as critical_region
from tactical_behaviour import TacticalBehavior
from ego_pose import EgoPose
from comm_msg import ComMsg
from motion import Motion



# --- Timing constants ---
TRAJ_DT = 0.001           # 1 ms trajectory update
DELTA_T = 0.1             # 100 ms tactical decision & rendering
OBPS_PERIOD = 0.03        # 50 ms OBPS message generation

# --- Vehicle dynamics
SIM_ADV_MAX_SPEED = 1.25
SIM_ADV_MAX_ACC   = 1.05
SIM_EGO_MAX_SPEED = 1.5
SIM_EGO_MAX_ACC   = 1.35


COM_BASE_DELAY = 120
COM_ADD_AOI = 0
COM_DELAY_MS = COM_BASE_DELAY + COM_ADD_AOI           # message timestamp delay in milli
COM_FAILURE_START_S = 0       # start of failure window (sec)
COM_FAILURE_LENGTH_S = 1.5    # length of failure window (sec)

# single-message queue for OBPS
obps_queue = queue.Queue(maxsize=1)


class OBPS:
    def __init__(self, 
                 delay_us=COM_DELAY_MS,
                 failure_start_s=COM_FAILURE_START_S,
                 failure_length_s=COM_FAILURE_LENGTH_S):
        
        self.delay_ns = delay_us * 1e6  # ms → ns
        self.failure_start_ns = failure_start_s * 1e9
        self.failure_length_ns = failure_length_s * 1e9
        self.base_time = None
        self.msg_id = 0
        self.last_msg = None

    def _get_noise(self) -> float:
        return 0.0  # placeholder for measurement noise

    def _make_msg(self, ts, adv_v, fx, fy):
        self.msg_id += 1
        return ComMsg(
            self.msg_id,
            ts - self.delay_ns,
            (fx + self._get_noise(), fy + self._get_noise()),
            (fx, fy - parameters.ADV_LENGTH),
            adv_v + self._get_noise()
        )

    def get_msg(self, adv_v, fx, fy):
        now = time.time_ns()
        if self.base_time is None:
            self.base_time = now
            return None

        # simulate failure window
        elapsed = now - self.base_time
        if (self.failure_start_ns <= elapsed < self.failure_start_ns + self.failure_length_ns):
            return None

        # generate on first valid call
        if self.last_msg is None:
            self.last_msg = self._make_msg(now, adv_v, fx, fy)
            return None

        # on subsequent calls, update timestamp & values
        self.last_msg = self._make_msg(now, adv_v, fx, fy)

        return self.last_msg
    
def is_ended(b: TacticalBehavior, ego_poly: shapely.Polygon, adv_poly: shapely.Polygon):

    if b.ego_d_front > b.ego_prediction.cr.cr_path.length - 1:
        return "PASSED", True, None

    cond1 = ego_poly.intersects(adv_poly)
    cond2 = ego_poly.crosses(adv_poly) 
    distance = shapely.distance(ego_poly, adv_poly)
    cond3 =  distance <= 0.1
    if cond1 or cond2 or cond3:
        #print(f"cond1:{cond1}, cond2:{cond2}, cond3:{cond3}")      
        return "CRASH", True, distance

    return None, False, None


def pub_ego_ref_speed(vel):
    pass
    #print("pub speed: {0}".format(vel))


def main(cfg: dict):
    # --- initialize geometry & modules ---
    cr_poly = critical_region.create_cr_polygon([
        parameters.CR_POINT_1,
        parameters.CR_POINT_2,
        parameters.CR_POINT_3,
        parameters.CR_POINT_4
    ])
    target_path = critical_region.create_path(
        parameters.ADV_PATH_START, parameters.ADV_PATH_END
    )
    target_cr = critical_region.CriticalRegion(target_path, cr_poly)
    target_cr.compute_critical_points()

    ego_path = critical_region.create_path(
        parameters.EGO_PATH_START, parameters.EGO_PATH_END
    )
    ego_cr = critical_region.CriticalRegion(ego_path, cr_poly)
    ego_cr.compute_critical_points()

    behaviour = TacticalBehavior(
        ego_reference_speed=parameters.EGO_REFERENCE_SPEED,
        ego_max_dec=parameters.EGO_MAX_DEC,
        ego_length=parameters.EGO_LENGTH,
        target_max_acc=cfg["ego_params"]["adv_max_acc"],
        target_max_speed=cfg["ego_params"]["adv_max_speed"],
        target_length=parameters.ADV_LENGTH,
        ego_critical_region=ego_cr,
        target_critical_region=target_cr
    )

    adv_motion = Motion(max_acc=SIM_ADV_MAX_ACC)
    ego_motion = Motion(max_acc=SIM_EGO_MAX_ACC)
    obps = OBPS(delay_us=cfg["obps"]["delay"], failure_length_s=cfg["obps"]["failure_len"], failure_start_s=cfg["obps"]["failure_start"])

    # state variables
    ego_ref = parameters.EGO_REFERENCE_SPEED
    adv_v = 0.0
    ego_v = 0.0
    front_target = None
    front_ego = None
    log_debug = {"target_front_d": [], "ego_front_d": [],
                 "ego_v": [], "target_v": []}

    # timers
    start_time = time.time()
    last_traj = start_time
    last_obps = start_time
    last_tactical = start_time

    # main loop
    while True:
        now = time.time()

        # 1) trajectory updates at 1ms
        while now - last_traj >= TRAJ_DT:
            adv_motion.update(SIM_ADV_MAX_SPEED, TRAJ_DT)
            d_t = adv_motion.get_total_displacement()
            front_target = target_path.interpolate(d_t + parameters.ADV_LENGTH/2)
            adv_v = adv_motion.v
            log_debug["target_front_d"].append(d_t)
            log_debug["target_v"].append(adv_v)

            ego_motion.update(ego_ref, TRAJ_DT)
            d_e = ego_motion.get_total_displacement()
            front_ego = ego_path.interpolate(d_e + parameters.EGO_LENGTH/2)
            ego_v = ego_motion.v
            log_debug["ego_front_d"].append(d_e)
            log_debug["ego_v"].append(ego_v)

            last_traj += TRAJ_DT

        # 2) OBPS message generation on its own period
        if now - last_obps >= OBPS_PERIOD:
            obps_msg = obps.get_msg(adv_v,
                               front_target.x, front_target.y)
            last_obps += OBPS_PERIOD
            
            if obps_queue.full():
                _ = obps_queue.get()
            obps_queue.put(obps_msg)

        # 3) tactical decision & rendering every DELTA_T
        if now - last_tactical >= DELTA_T:
            # fetch latest msg (non-blocking)
            try:
                msg = obps_queue.get_nowait()
            except queue.Empty:
                msg = None

            # build ego pose
            pose = EgoPose(
                front_x=front_ego.x, front_y=front_ego.y,
                rear_x=front_ego.x - parameters.EGO_LENGTH,
                rear_y=front_ego.y - parameters.EGO_LENGTH,
                vel_x=ego_v, vel_y=0,
                ego_max_acc=parameters.EGO_MAX_ACC
            )

            #msg = get_static_message()
            action = behaviour.decision(msg, pose)
            ego_ref = behaviour.action_to_speed(action)
            behaviour.log()

            adv_center = target_path.interpolate(
                adv_motion.get_total_displacement()
            )
            ego_center = ego_path.interpolate(
                ego_motion.get_total_displacement()
            )

            ego_front_x   =  ego_center.x - parameters.EGO_LENGTH/2
            ego_rear_x    =  ego_center.x + parameters.EGO_LENGTH/2
            ego_width_rx  =  ego_center.y + parameters.EGO_WIDTH/2
            ego_width_lx  =  ego_center.y - parameters.EGO_WIDTH/2

            ego_poly = shapely.Polygon([
                                (ego_front_x, ego_width_lx), 
                                (ego_front_x, ego_width_rx), 
                                (ego_rear_x, ego_width_rx), 
                                (ego_rear_x, ego_width_lx)])
            
            adv_front_y   =  adv_center.y + parameters.ADV_LENGTH/2
            adv_rear_y    =  adv_center.y - parameters.ADV_LENGTH/2
            adv_width_rx  =  adv_center.x + parameters.ADV_WIDTH/2
            adv_width_lx  =  adv_center.x - parameters.ADV_WIDTH/2

            adv_poly = shapely.Polygon([
                                (adv_width_lx, adv_front_y), 
                                (adv_width_rx, adv_front_y), 
                                (adv_width_rx, adv_rear_y), 
                                (adv_width_lx, adv_rear_y)])  
            
            # check termination
            cause, ended, distance = is_ended(behaviour,
                                    ego_poly, 
                                    adv_poly)

            if ended:
                pub_ego_ref_speed(0.0)
                #TestExpLog(behaviour.data_log,log_debug, cause).write_to_file()                           
                break


            last_tactical += DELTA_T

    return cause, distance


   
def run_experiment(args):
    base_delay, add_aoi, fail_start, fail_len, adv_max_acc, adv_max_speed = args
    total_delay = base_delay + add_aoi

    cfg = {
        "obps":{
            "delay": total_delay,
            "failure_start": fail_start,
            "failure_len": fail_len
        },
        "ego_params":{
            "adv_max_acc": adv_max_acc, 
            "adv_max_speed": adv_max_speed
        }
    }

    outcome, distance = main(cfg)

    return {"cfg": cfg, "outcome": outcome, "distance": distance}

if __name__ == '__main__':

    #out = run_experiment((60, 0, 0, 0))
    #exit(0)

    # --- OBPS parameterization ---
    COM_BASE_DELAY_LIST     = [60, 80]
    COM_ADD_AOI_LIST        = list(range(0, 1100, 100))
    COM_FAILURE_START_LIST  = [round(x, 1) for x in np.arange(0, 2.5, 0.1)]       # start of failure window (sec)
    COM_FAILURE_LEN_LIST    = [0, 0.5, 1.1, 1.2, 1.3, 1.5, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.5, 10]    # length of failure window (sec)
    ADV_MAX_ACC_LIST        = [1.05]
    ADV_MAX_SPEED_LIST      = [1.25]   
   
    
    multiprocessing.freeze_support()

    all_args = list(itertools.product(
        COM_BASE_DELAY_LIST,
        COM_ADD_AOI_LIST,
        COM_FAILURE_START_LIST,
        COM_FAILURE_LEN_LIST,
        ADV_MAX_ACC_LIST,
        ADV_MAX_SPEED_LIST
    ))
    total_runs = len(all_args)

    # create pool
    with multiprocessing.Pool() as pool:
        # imap_unordered yields results as they come in
        results = []
        for result in tqdm(
            pool.imap_unordered(run_experiment, all_args),
            total=total_runs,
            desc="Running simulations",
            unit="run"
        ):
            results.append(result)

    # dump to JSON
    file_name = f'results_{time.time()}.json'
    body ={"out":results}
    with open(file_name, 'w') as fp:
        json.dump(body, fp, indent=2)

    print(f"[INFO] Completed {total_runs} runs — results saved to {file_name}")
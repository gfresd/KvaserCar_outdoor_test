import time
import queue
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

# --- Timing constants ---
TRAJ_DT = 0.001           # 1 ms trajectory update
DELTA_T = 0.1             # 100 ms tactical decision & rendering
OBPS_PERIOD = 0.03        # 50 ms OBPS message generation

# --- OBPS configuration constants ---
COM_BASE_DELAY = 120
COM_ADD_AOI = 800
COM_DELAY_MS = COM_BASE_DELAY + COM_ADD_AOI           # message timestamp delay in milli
COM_FAILURE_START_S = 0       # start of failure window (sec)
COM_FAILURE_LENGTH_S = 1.5    # length of failure window (sec)

# single-message queue for OBPS
obps_queue = queue.Queue(maxsize=1)




class OBPS:
    def __init__(self, delay_us=COM_DELAY_MS,
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
        if (self.failure_start_ns <= elapsed <
                self.failure_start_ns + self.failure_length_ns):
            return None

        # generate on first valid call
        if self.last_msg is None:
            self.last_msg = self._make_msg(now, adv_v, fx, fy)
            return None

        # on subsequent calls, update timestamp & values
        self.last_msg = self._make_msg(now, adv_v, fx, fy)

        return self.last_msg
    

def is_ended(b: TacticalBehavior, front_ego, front_target):
    if b.ego_d_front > b.ego_prediction.cr.cr_path.length - 1:
        return "PASSED", True

    if shapely.distance(front_target, front_ego) < 0.2:
        #return "CRASH", True
        pass

    return None, False


def pub_ego_ref_speed(vel):
    print("pub speed: {0}".format(vel))


def get_static_message():

    return ComMsg(
        0,
        time.time_ns() - 10*10e-6,
        (0, parameters.ADV_PATH_START[1]),
        (0, parameters.ADV_PATH_START[1] - parameters.ADV_LENGTH),
        0
    )

def main():
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
        target_max_acc=parameters.ADV_MAX_ACC,
        target_max_speed=parameters.ADV_MAX_SPEED,
        target_length=parameters.ADV_LENGTH,
        ego_critical_region=ego_cr,
        target_critical_region=target_cr
    )

    adv_motion = Motion(max_acc=parameters.ADV_MAX_ACC)
    ego_motion = Motion(max_acc=parameters.EGO_MAX_ACC)
    obps = OBPS()

    # plotter setup
    plotter = TrajectoryPlotter()
    plotter.set_critical_region([
        parameters.CR_POINT_1,
        parameters.CR_POINT_2,
        parameters.CR_POINT_3,
        parameters.CR_POINT_4
    ])
    plotter.set_ego_path(
        parameters.EGO_PATH_START, parameters.EGO_PATH_END
    )
    plotter.set_adv_path(
        parameters.ADV_PATH_START, parameters.ADV_PATH_END
    )

    # state variables
    ego_ref = parameters.EGO_REFERENCE_SPEED
    adv_v = 0.0
    ego_v = 0.0
    front_target = None
    front_ego = None
    last_msg = None
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
            adv_motion.update(parameters.ADV_MAX_SPEED, TRAJ_DT)
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

            # check termination
            cause, ended = is_ended(behaviour,
                                     front_ego, front_target)
            if ended:
                print(f"Experiment terminated: {cause}")
                pub_ego_ref_speed(0.0)
                TestExpLog(behaviour.data_log,
                           log_debug, cause).write_to_file()
                break

            # update and render plot
            adv_center = target_path.interpolate(
                adv_motion.get_total_displacement()
            )
            ego_center = ego_path.interpolate(
                ego_motion.get_total_displacement()
            )
            plotter.update_ego_pose(ego_center.x,
                                     ego_center.y, 0)
            plotter.update_adv_pose(adv_center.x,
                                     adv_center.y,
                                     -math.pi/2)
            if behaviour.target_prediction.d_front != -1:
                xf, yf = behaviour.target_prediction.get_coords_of_projected_front()
                plotter.set_tactical_front(xf, yf)

            plotter.render()
            last_tactical += DELTA_T


if __name__ == '__main__':
    main()

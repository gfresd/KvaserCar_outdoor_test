import shapely
from critical_region import CriticalRegion
import math

class EgoPrediction:

    NO_TIME_TO_CR = -1

    def __init__(self,
                 reference_speed: float,
                 ego_max_break,
                 ego_len: float,
                 critical_region: CriticalRegion):
        """

        :param reference_speed: in [m/s]
        :param ego_max_break: in [m/s]
        :param ego_len: in [m]
        :param critical_region:
        """
        self.reference_speed = reference_speed
        self.ego_max_break = ego_max_break
        self.ego_len = ego_len
        self.cr = critical_region
        self.gt_dist_to_cr = -1
        self.gt_d_front = -1
        self.gt_d_rear = -1
        self.v_delta = 0.1
        self.eps = 0.01

    
    def at_max_speed(self, target_vel)-> bool:
        cond1 = math.fabs(target_vel - self.reference_speed) < self.v_delta
        cond2 = target_vel > self.reference_speed
        return cond1 or cond2
       
    def _get_time_to(self, vel, acc, distance):
        # check if we are at max speed
        if self.at_max_speed(vel):
            return distance / self.reference_speed
        
        # Treat extremely small velocities as zero
        if abs(vel) < self.eps:
            vel = 0.0

        # if we are not at max speed we should calculate the time piece wise
        d1 = ((self.reference_speed ** 2) - (vel**2)) / (2 * acc)
        if distance > d1:
            #accelerated motion
            t1 = (self.reference_speed - vel) / acc
            #constant motion
            d2 = distance - d1
            t2 = d2 / self.reference_speed
            return t1 + t2
        else:
            # only accelerated motion, that means that while moving up to distance we never reach max speed
            # discriminant V*v - 4*0.5*a*(-distance) -> V*v + 4*0.5*a*(distance)
            discriminant = vel ** 2 - 2 * acc * (-distance)
            t = (-vel + math.sqrt(discriminant)) / acc
            return t

    
    def get_time_to_leave_cr(self, relative_pos: CriticalRegion.Position, current_vel:float , accel: float) -> float:
        t = self.NO_TIME_TO_CR

        if relative_pos == CriticalRegion.Position.BEFORE_CR or relative_pos == CriticalRegion.Position.INSIDE_CR:
            #get distance to leave the CR
            distance = math.fabs(self.cr.cf_orig_d - self.gt_d_rear)
            t = self._get_time_to(current_vel, accel, distance)

        return t
    

    def get_time_to_cr(self,
                       relative_pos: CriticalRegion.Position,
                       current_vel:float,
                       accel:float):
        """
               
        
        :relative_pos:
        :param current_vel:
        :param accel:
        :return:
        """
        # This time only exists if we are before the region
        target_time = self.NO_TIME_TO_CR

        if relative_pos == CriticalRegion.Position.BEFORE_CR:
            # get distance to enter the CR
            # we are before the CN point!
            distance = math.fabs(self.cr.cn_orig_d - self.gt_d_front)
            target_time = self._get_time_to(current_vel, accel, distance)

        return target_time 

    
    def get_dist_to_cr(self, relative_pos: CriticalRegion.Position):
        if relative_pos == CriticalRegion.Position.UNKNOWN:
            return -1
        
        if relative_pos == CriticalRegion.Position.AFTER_CR or CriticalRegion.Position.INSIDE_CR:
            return 0
        
        if relative_pos == CriticalRegion.Position.BEFORE_CR:
            return max(0 , self.cr.cn_orig_d - self.d_front) 


    def _get_relative_position(self, front_d, rear_d):
        if front_d < self.cr.cn_orig_d:
            return CriticalRegion.Position.BEFORE_CR

        if rear_d > self.cr.cf_orig_d:
            return CriticalRegion.Position.AFTER_CR

        if self.cr.cn_orig_d <= front_d <= self.cr.cf_orig_d or self.cr.cn_orig_d <= rear_d <= self.cr.cf_orig_d:
            return CriticalRegion.Position.INSIDE_CR

    def get_current_pos(self, front: shapely.Point):
        self.gt_d_front, self.gt_d_rear = self.project_to_path(front, 0)
        current_pos = self._get_relative_position(self.gt_d_front, self.gt_d_rear)
        return self.gt_d_front, self.gt_d_rear, current_pos

    def project_to_path(self, front: shapely.Point, displacement: float):
        # project the detected front on the critical path and find its distance
        # then add the displacement
        d_front = shapely.line_locate_point(self.cr.cr_path, front) + displacement

        # find the rear point distance relative to the front
        d_rear = d_front - self.ego_len

        return d_front, d_rear

    def get_dist_and_time_to_cr(self, vel: float, front: shapely.Point):

        gt_d_front, gt_d_rear, gt_pos = self.get_current_pos(front)
        if gt_pos == CriticalRegion.Position.BEFORE_CR:
            dist_to_cr = self.cr.cn_orig_d - gt_d_front
            # we need to cap the max value for too small values of v
            if vel == 0:
                tt_cr = 100000000
            else:
                tt_cr = min(dist_to_cr / vel, dist_to_cr / 0.5)
        else:
            dist_to_cr = -1
            tt_cr = -1

        return gt_d_front, dist_to_cr, tt_cr

    def get_predicted_positions(self, vel: float, acc: float, time: float, front: shapely.Point):

        # check the GO displacement
        front_p = shapely.Point(front)
        shapely.prepare(front_p)

        d_go = self.predict_go_displacement(vel, acc, time)
        d_go_front, d_go_rear = self.project_to_path(front_p, d_go)
        # check the position when using the current speed and target_speed for prediction
        pos_go = self._get_relative_position(d_go_front, d_go_rear)

        # check the BREAK displacement
        d_break = self.predict_break_displacement(vel, time)
        d_break_front, d_break_rear = self.project_to_path(front_p, d_break)
        # check the position if we start to break
        pos_break = self._get_relative_position(d_break_front, d_break_rear)

        return pos_go, pos_break



    def predict_go_displacement(self, velocity: float, accel: float, t: float):
        """
        Predict where ego will be in the given time t
        :param accel: current acceleration
        :param velocity: the current velocity
        :param t: time to perform the manoeuvre
        :return: location after time t, distance driven in time t
        """

        if velocity < self.reference_speed and accel > 0:
            t_x = (self.reference_speed - velocity) / accel
            if t - t_x >= 0:
                d_1 = velocity * t_x + 0.5 * accel * t_x ** 2
                d_2 = self.reference_speed * (t - t_x)
                d = d_1 + d_2
            else:
                d = velocity * t + 0.5 * accel * t ** 2
        else:
            # steady state velocity
            d = velocity * t

        return d


    def predict_break_displacement(self, velocity: float, t: float):
        """

        :param velocity: the current velocity
        :param t: time to perform the manoeuvre
        :return: location after time t, distance driven in time t
        """

        d = velocity * t - 0.5 * self.ego_max_break * (t ** 2)
        d = max(0.0, d)

        return d
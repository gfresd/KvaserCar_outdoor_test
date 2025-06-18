
import shapely
import math
from tactical_node.critical_region import CriticalRegion

class TargetPrediction:

    NO_TIME_TO_CR = -1

    def __init__(self,
                 target_id,
                 critical_region: CriticalRegion,
                 max_speed
                 ):

        self.id = target_id
        self.cr = critical_region
        self.target_pos = CriticalRegion.Position.UNKNOWN
        self.d_front = -1
        self.d_rear = -1
        self.dist_to_cr = -1
        self.max_speed = max_speed
        self.v_delta = 0.1
        self.eps = 0.01

    def at_max_speed(self, target_vel)-> bool:
        cond1 = math.fabs(target_vel - self.max_speed) < self.v_delta
        cond2 = target_vel > self.max_speed
        return cond1 or cond2
       
    def _get_time_to(self, vel, acc, distance):
        # check if we are at max speed
        if self.at_max_speed(vel):
            return distance / self.max_speed
        
        # Treat extremely small velocities as zero
        if abs(vel) < self.eps:
            vel = 0.0

        # if we are not at max speed we should calculate the time piece wise
        d1 = ((self.max_speed ** 2) - (vel**2)) / (2 * acc)
        if distance > d1:
            #accelerated motion
            t1 = (self.max_speed - vel) / acc
            #constant motion
            d2 = distance - d1
            t2 = d2 / self.max_speed
            return t1 + t2
        else:
            # only accelerated motion, that means that while moving up to distance we never reach max speed
            # discriminant V*v - 4*0.5*a*(-distance) -> V*v + 4*0.5*a*(distance)
            discriminant = vel ** 2 - 2 * acc * (-distance)
            t = (-vel + math.sqrt(discriminant)) / acc
            return t
           

    def project_to_path(self, front: shapely.Point, target_length: float, displacement: float):
        # project the detected front on the critical path and find its distance
        d_front = shapely.line_locate_point(self.cr.cr_path, front) + displacement

        # find the rear point on the critical path
        d_rear = d_front - target_length
        d_rear = d_rear if d_rear > 0 else -1
        return d_front, d_rear
    

    def get_predicted_aoi_displacement(self, delta_time: float, current_vel:float, target_acc:float):
        if self.at_max_speed(current_vel):
            return self.max_speed * delta_time
        else:
            t_x = (self.max_speed - current_vel) / target_acc
            if delta_time - t_x >= 0:
                d_1 = current_vel * t_x + 0.5 * target_acc * t_x ** 2
                d_2 = self.max_speed * (delta_time - t_x)
                d = d_1 + d_2
                return d
            else:
                d = current_vel * delta_time + 0.5 * target_acc * delta_time ** 2
                return d       


    def _get_relative_position(self, front_d, rear_d):
        if front_d < self.cr.cn_orig_d:
            return CriticalRegion.Position.BEFORE_CR

        if rear_d > self.cr.cf_orig_d:
            return CriticalRegion.Position.AFTER_CR

        if self.cr.cn_orig_d <= front_d <= self.cr.cf_orig_d or self.cr.cn_orig_d <= rear_d <= self.cr.cf_orig_d:
            return CriticalRegion.Position.INSIDE_CR 



    def get_position(self,
                    aoi:float,
                    target_vel:float,
                    target_acc:float,
                    target_length:float,
                    front: shapely.Point):
        # Compensate for the AoI by calculating the displacement that the target makes
        # in delta_t = (current_t - aoi)
        # Consider that the target is at aoi_comp_pos = (observed position + compensated displacement)
        
        displacement = self.get_predicted_aoi_displacement(aoi, target_vel, target_acc)

        self.d_front, self.d_rear = self.project_to_path(front, target_length, displacement)

        self.dist_to_cr = max(0, self.cr.cn_orig_d - self.d_front)

        self.target_pos = self._get_relative_position(self.d_front, self.d_rear)

        return self.target_pos
    

    def get_dist_to_cr(self, relative_pos: CriticalRegion.Position):
        if relative_pos == CriticalRegion.Position.UNKNOWN:
            return -1
        
        if relative_pos == CriticalRegion.Position.AFTER_CR or CriticalRegion.Position.INSIDE_CR:
            return 0
        
        if relative_pos == CriticalRegion.Position.BEFORE_CR:
            return max(0 , self.cr.cn_orig_d - self.d_front)


    def get_time_to_cr(self,
                       relative_pos: CriticalRegion.Position,
                       current_vel:float,
                       target_acc:float):
        """
        

        :param target_acc:
        :param target_vel:
        :relative_pos:
        :return:
        """
        # This time only exists if we are before the region
        target_time = self.NO_TIME_TO_CR

        if relative_pos == CriticalRegion.Position.BEFORE_CR:
            # get distance to enter the CR
            # we are before the CN point!
            distance = math.fabs(self.cr.cn_orig_d - self.d_front)
            target_time = self._get_time_to(current_vel, target_acc, distance)

        return target_time  



    def get_time_to_leave_cr(self, target_pos, current_vel, target_acc) -> float:

        target_time = self.NO_TIME_TO_CR

        if target_pos == CriticalRegion.Position.BEFORE_CR or target_pos == CriticalRegion.Position.INSIDE_CR:
            #get distance to leave the CR
            distance = math.fabs(self.cr.cf_orig_d - self.d_rear)
            target_time = self._get_time_to(current_vel, target_acc, distance)

        return target_time
    
    def get_coords_of_projected_front(self):
        front_point = self.cr.cr_path.interpolate(self.d_front)
        return front_point.x, front_point.y





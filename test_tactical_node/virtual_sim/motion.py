import math
from typing import List
import numpy as np

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
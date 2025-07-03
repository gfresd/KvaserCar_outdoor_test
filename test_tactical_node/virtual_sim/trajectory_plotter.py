
import math
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
import shapely


class TrajectoryPlotter:
    def __init__(self,
                 length: float = 0.720,
                 width: float = 0.515,
                 buffer_size: int = 1000,
                 xlim: tuple = (-6, 6),
                 ylim: tuple = (-6, 6),
                 pause_time: float = 0.05):
        """
        Initializes the plotting window and data buffers.

        Args:
            length: Vehicle length [m].
            width: Vehicle width [m].
            buffer_size: Max number of trajectory points to keep.
            xlim: X-axis limits.
            ylim: Y-axis limits.
            pause_time: Time to pause on each draw (s).
        """
        # trajectory buffers
        self.ego_buf = deque(maxlen=buffer_size)
        self.adv_buf = deque(maxlen=buffer_size)
        self.tac_front = None

        # path endpoints
        self.ego_path_start = None
        self.ego_path_end = None
        self.adv_path_start = None
        self.adv_path_end = None


        # critical region
        self.crit_region = None

        # vehicle dims
        self.length = length
        self.width = width
        self.pause_time = pause_time

        # set up figure
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 12))
        self.ax.set_xlabel('X position [m]')
        self.ax.set_ylabel('Y position [m]')
        self.ax.set_title('Live Trajectories with Critical Region')
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)

        # plot elements
        self.ego_traj_line, = self.ax.plot([], [], 'b-', label='Ego trajectory')
        self.adv_traj_line, = self.ax.plot([], [], 'r-', label='Adv trajectory')
        self.ego_path_line, = self.ax.plot([], [], 'b--', label='Ego path')
        self.adv_path_line, = self.ax.plot([], [], 'r--', label='Adv path')

        self.crit_patch = Polygon(np.empty((0, 2)), closed=True, fc='black', alpha=0.3)
        self.ego_patch = Rectangle((-length / 2, -width / 2), length, width, fc='blue', alpha=0.6)
        self.adv_patch = Rectangle((-length / 2, -width / 2), length, width, fc='red', alpha=0.6)
        self.tac_patch = Rectangle((-length, -width / 2), length, width,
                                   fc='none', ec='green', ls=':', lw=2, alpha=0.8)
        
        #initialize off path
        trans = (plt.matplotlib.transforms.Affine2D()
                     .rotate_around(0, 0, 0)
                     .translate(-5, -5)
                     + self.ax.transData)
        self.tac_patch.set_transform(trans)

        self.ego_poly_patch = None  
        self.adv_poly_patch = None  


        # add to axes
        self.ax.add_patch(self.crit_patch)
        #self.ax.add_patch(self.ego_patch)
        #self.ax.add_patch(self.adv_patch)
        self.ax.add_patch(self.tac_patch)
        self.ax.legend()
        self.fig.canvas.draw()

    def set_critical_region(self, vertices: list):
        """
        vertices: list of (x,y) tuples
        """
        self.crit_region = vertices
        self.crit_patch.set_xy(np.array(vertices))

    def set_ego_path(self, start: tuple, end: tuple):
        self.ego_path_start = start
        self.ego_path_end = end
        xs = [start[0], end[0]]
        ys = [start[1], end[1]]
        self.ego_path_line.set_data(xs, ys)

    def set_adv_path(self, start: tuple, end: tuple):
        self.adv_path_start = start
        self.adv_path_end = end
        xs = [start[0], end[0]]
        ys = [start[1], end[1]]
        self.adv_path_line.set_data(xs, ys)

    def update_polygons(self, ego_poly: shapely.Polygon, adv_poly: shapely.Polygon):
        """
        Given two shapely Polygons (in data�coordinates),
        create or update their Matplotlib patches so you
        can see exactly the same footprints you computed.
        """
        coords_e = list(ego_poly.exterior.coords)
        coords_a = list(adv_poly.exterior.coords)

        # Ego footprint
        if self.ego_poly_patch is None:
            self.ego_poly_patch = Polygon(
                coords_e, closed=True,
                edgecolor='blue', facecolor='blue', alpha=0.6
            )
            self.ax.add_patch(self.ego_poly_patch)
        else:
            self.ego_poly_patch.set_xy(coords_e)

        # Adv footprint
        if self.adv_poly_patch is None:
            self.adv_poly_patch = Polygon(
                coords_a, closed=True,
                edgecolor='red', facecolor='red', alpha=0.6
            )
            self.ax.add_patch(self.adv_poly_patch)
        else:
            self.adv_poly_patch.set_xy(coords_a)

    def update_ego_pose(self, x: float, y: float, yaw: float):
        """
        Append a new ego pose and update the patch and trajectory.
        """
        self.ego_buf.append((x, y, yaw))
        xs, ys, _ = zip(*self.ego_buf)
        self.ego_traj_line.set_data(xs, ys)
        xe, ye, yeaw = self.ego_buf[-1]
        trans = (plt.matplotlib.transforms.Affine2D()
                 .rotate_around(0, 0, yeaw)
                 .translate(xe, ye)
                 + self.ax.transData)
        self.ego_patch.set_transform(trans)

    def update_adv_pose(self, x: float, y: float, yaw: float):
        """
        Append a new adversary pose and update the patch and trajectory.
        """
        self.adv_buf.append((x, y, yaw))
        xs, ys, _ = zip(*self.adv_buf)
        self.adv_traj_line.set_data(xs, ys)
        xa, ya, ayaw = self.adv_buf[-1]
        trans = (plt.matplotlib.transforms.Affine2D()
                 .rotate_around(0, 0, ayaw)
                 .translate(xa, ya)
                 + self.ax.transData)
        self.adv_patch.set_transform(trans)

    def set_tactical_front(self, x: float, y: float):
        """
        Set the tactical front target and update its patch.
        """
        self.tac_front = (x, y)
        if self.adv_buf:
            _, _, ayaw = self.adv_buf[-1]
            trans = (plt.matplotlib.transforms.Affine2D()
                     .rotate_around(0, 0, ayaw)
                     .translate(x, y)
                     + self.ax.transData)
            self.tac_patch.set_transform(trans)
        else:
            trans = (plt.matplotlib.transforms.Affine2D()
                     .rotate_around(0, 0, -math.pi/2)
                     .translate(x, y)
                     + self.ax.transData)
            self.tac_patch.set_transform(trans)


    def render(self):
        """
        Draw the current state and pause briefly for interactive update.
        """
        #self.fig.canvas.draw()
        #plt.pause(self.pause_time)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def clear(self):
        """
        Clear buffers and reset plot elements to empty.
        """
        self.ego_buf.clear()
        self.adv_buf.clear()
        self.ego_traj_line.set_data([], [])
        self.adv_traj_line.set_data([], [])
        self.ego_path_line.set_data([], [])
        self.adv_path_line.set_data([], [])
        self.crit_patch.set_xy(np.empty((0, 2)))
        self.ego_patch.set_transform(self.ax.transData)
        self.adv_patch.set_transform(self.ax.transData)
        self.tac_patch.set_transform(self.ax.transData)


from enum import IntEnum
from typing import List

import shapely

import plotly.graph_objects as go

def create_cr_polygon(points: List[shapely.Point]) -> shapely.Polygon:
    poly = shapely.Polygon(points)
    shapely.prepare(poly)
    return poly

def create_path(start_point: shapely.Point, end_point: shapely.Point) -> shapely.LineString:
    path = shapely.LineString([start_point, end_point])
    shapely.prepare(path)
    return path

class CriticalRegion:
    MARGIN = 0.2

    class Position(IntEnum):
        BEFORE_CR = 0
        AFTER_CR = 10
        INSIDE_CR = 20
        UNKNOWN = -10


    #                critical region
    #                 (P4)-----(P1)
    #                  |         |
    # path_start--->>(*cn)    (*cf)-->>path_end
    #                  |         |
    #                 (P3)_____(P2)
    #
    def __init__(self, path: shapely.LineString, critical_region: shapely.Polygon):
        self.cn = None
        self.cf = None
        self.cr_path = path
        self.cr = critical_region
        self.cn_orig_d = -1
        self.cf_orig_d = -1


    def compute_critical_points(self,):
        inter = shapely.intersection(self.cr_path, self.cr.boundary)

        if inter is None:
            raise Exception("Critical Region path intersection not found")

        # we assume that the intersections are LineStrings
        if inter.geom_type != "MultiPoint":
            raise Exception("Problem critical region computation")

        # multipoints are not given in order! (Checked with plot)
        # find who is c_near and cn_orig_d  AND c_far and cf_orig_d
        d1 = shapely.line_locate_point(self.cr_path, inter.geoms[0])
        d2 = shapely.line_locate_point(self.cr_path, inter.geoms[1])
        # temp is [(d1, point1), (d2, point2)]
        temp = [(d1, inter.geoms[0]), (d2, inter.geoms[1])]
        temp.sort(key=lambda x: x[0])

        self.cn = temp[0][1]
        self.cn_orig_d = temp[0][0]
        self.cf = temp[1][1]
        self.cf_orig_d = temp[1][0]


    def plot_regions(self, header =""):
        x, y = self.cr.boundary.xy
        trace_0 = go.Scatter(
            x=x.tolist(),
            y=y.tolist(),
            mode="lines",
            name="critical region",
            line={"color": "blue"},
        )

        x, y = self.cr_path.xy
        trace_1 = go.Scatter(
            x=x.tolist(),
            y=y.tolist(),
            mode="lines",
            name="cr_path",
            line={"color": "red"},
        )

        trace_5 = go.Scatter(
            x=[self.cf.x],
            y=[self.cf.y],
            mode="markers",
            name="cf",
            marker={"color": "brown", "size":8}
        )

        trace_6 = go.Scatter(
            x=[self.cn.x],
            y=[self.cn.y],
            mode="markers",
            name="cn",
            marker={"color": "black", "size":8}
        )

        cf_margin = self.cr_path.interpolate(self.cf_orig_d + CriticalRegion.MARGIN)
        trace_7 = go.Scatter(
            x=[cf_margin.x],
            y=[cf_margin.y],
            mode="markers",
            name="cf + {0} margin".format(CriticalRegion.MARGIN),
            marker={"color": "gold", "size": 8}
        )

        cn_margin = self.cr_path.interpolate(self.cn_orig_d - CriticalRegion.MARGIN)
        trace_8 = go.Scatter(
            x=[cn_margin.x],
            y=[cn_margin.y],
            mode="markers",
            name="cn - {0} margin".format(CriticalRegion.MARGIN),
            marker={"color": "gold", "size": 8}
        )

        traces = [trace_0, trace_1, trace_5, trace_6, trace_7, trace_8]

        layout = go.Layout(
            title="{0} Critical Region".format(header),
            hovermode='x unified'
        )

        fig = go.Figure(data=traces, layout=layout)
        fig.show()
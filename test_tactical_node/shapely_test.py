import shapely
import math

LEN = 1

start_x = 1
line = shapely.LineString([[-4.0, 0], [4.0, 0]])
front_p = shapely.Point([start_x + LEN/2, 0])
rear_p = shapely.Point([start_x - LEN/2, 0])

crn_p = shapely.Point([-0.5, 0])
crf_p = shapely.Point([0.5, 0])

d_front = shapely.line_locate_point(line, front_p)
d_rear = shapely.line_locate_point(line, rear_p)
d_crn = shapely.line_locate_point(line, crn_p)
d_crf = shapely.line_locate_point(line, crf_p)

calc_d_rear = d_front - LEN

dist_to_leave      = math.fabs(d_crf - d_rear)
calc_dist_to_leave = math.fabs(d_crf - calc_d_rear)

print(d_front)
print(d_rear)
print(d_crn)
print(d_crf)
print(calc_d_rear)

print(f'dist to leave 1) {dist_to_leave}')
print(f'dist to leave 2) {calc_dist_to_leave}')

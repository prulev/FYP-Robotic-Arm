#! /usr/bin/env python
from frankx import Robot
from frankx import Affine
from frankx import LinearRelativeMotion, JointMotion, LinearMotion, WaypointMotion, Waypoint
import numpy as np
import copy
import time


get_now = time.perf_counter


def sleep(duration, get_now=get_now):
    now = get_now()
    end = now + duration
    while now < end:
        now = get_now()


step = 0.1  # time step (s)
s = 0.5
max_speed = 1  # end point speed (m/s)
max_size = 0.4
hz = 250
num_pts = 100
num_exe = int((num_pts-1) * hz * s)
spline_mat = 0.5 * np.array([0, -1, 2, -1,
                             2, 0, -5, 3,
                             0, 1, 4, -3,
                             0, 0, -1, 1]).reshape(4, 4)  # Catmull-Rom splines
poly_mat = np.array([1, 0, 0, 0, 0, 0,
                     1, s, s ** 2, s ** 3, s ** 4, s ** 5,
                     0, 1, 0, 0, 0, 0,
                     0, 1, 2 * s, 3 * s ** 2, 4 * s ** 3, 5 * s ** 4,
                     0, 0, 2, 0, 0, 0,
                     0, 0, 2, 6 * s, 12 * s ** 2, 20 * s ** 3]).reshape(6, 6)
inv_poly = np.linalg.inv(poly_mat)
num_sample = int(hz * s)
t = np.arange(num_sample) / num_sample
basis = np.stack([np.ones_like(t), t, t ** 2, t ** 3], axis=0)
basis_v = np.stack([np.zeros_like(t), np.ones_like(t), 2 * t, 3 * t ** 2], axis=0)

with open('traj.txt') as f:
    lines = f.readlines()
x, y = list(), list()
for line in lines[1:]:
    line = line.replace(' ', '').replace('\n', '').split('\t')
    x.append(float(line[1]))
    y.append(float(line[2]))
x = np.asarray(x)[:num_pts]
y = np.asarray(y)[:num_pts]
x = x +0.07
y = y +0.37
rescale = max_size / 4
x = rescale * x
y = rescale * y
pts = np.stack([x, y], axis=1)
diff_pts = pts[1:] - pts[:-1]

robot = Robot("192.168.1.2")
robot.recover_from_errors()
robot.set_dynamic_rel(0.05)
robot.acceleration_rel = 0.3
robot.velocity_rel = 0.7
m0 = JointMotion([-0.10950471, -0.24605912, 0.10481801, -2.35875043, 0.0364012, 2.11087105, 0.7573013])
robot.move(m0)
a0 = robot.current_pose()
a = Affine(a0)
a.x, a.y = a.x + pts[0, 0], a.y + pts[0, 1]
current_pose = robot.current_pose()
ms = WaypointMotion([Waypoint(a)])
robot.move(ms)
# wm = [Waypoint(a0)]
x, y = list(), list()
bo = get_now()
for i in range(1, pts.shape[0]):
    bi = get_now()
    dx, dy = pts[i]
    a = Affine(a0)
    a.x, a.y = a.x + dx, a.y + dy
    ms = WaypointMotion([Waypoint(a, Waypoint.Absolute)])
    # ms.set_next_waypoint(Waypoint(a, Waypoint.Absolute))
    # wm.append(Waypoint(Affine(dx, dy, 0), Waypoint.Relative))
    robot.move(ms)
    current_pose = robot.current_pose()
    x.append(current_pose.x)
    y.append(current_pose.y)
    ei = get_now()
    print(ei-bi)
    # sleep(0.2)
eo = get_now()
print(eo-bo)
# ms = WaypointMotion(wm)
sleep(1.0)
current_pose = robot.current_pose()
import matplotlib.pyplot as plt
plt.plot(pts.T[0], pts.T[1])
plt.plot(np.asarray(x)-a0.x, np.asarray(y)-a0.y)
plt.show()

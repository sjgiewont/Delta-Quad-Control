import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import timeit
from inverseKinematics import inverseKinematics
from servoControl import *

import main


def move_to_pos(position_R, position_L, index):
    leg_1_thetas = inverseKinematics(position_R[index[0]])
    leg_2_thetas = inverseKinematics(position_R[index[1]])
    leg_3_thetas = inverseKinematics(position_L[index[2]])
    leg_4_thetas = inverseKinematics(position_L[index[3]])

    servo_1_values = angleToServoValue(leg_1_thetas, 1)
    servo_2_values = angleToServoValue(leg_2_thetas, 2)
    servo_3_values = angleToServoValue(leg_3_thetas, 3)
    servo_4_values = angleToServoValue(leg_4_thetas, 4)

    serialSend_All(servo_1_values, servo_2_values, servo_3_values, servo_4_values)

# def step(curr_pos, new_pos):
#     #parabola function between 2 points
#     start_pt = np.array([curr_pos[0], curr_pos[1], curr_pos[2]])
#     end_pt = np.array([new_pos[0], new_pos[1], new_pos[2]])
#
#     step_height = 3     #the height difference between steps, relative offset from start Z
#
#     mid = (start_pt + end_pt) / float(2)
#
#     mid_pt = np.array([mid[0], mid[1], mid[2]+step_height])
#
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#
#     # for t in np.arange(0, 1, 0.001):
#     t = np.arange(0, 1, 0.001)
#
#     start = timeit.default_timer()
#
#     x_pt = start_pt[0] - (t * (3*start_pt[0] - 4*mid_pt[0] + end_pt[0])) + 2*t*t*(start_pt[0] - 2*mid_pt[0] + end_pt[0])
#     y_pt = start_pt[1] - (t * (3*start_pt[1] - 4*mid_pt[1] + end_pt[1])) + 2*t*t*(start_pt[1] - 2*mid_pt[1] + end_pt[1])
#     z_pt = start_pt[2] - (t * (3*start_pt[2] - 4*mid_pt[2] + end_pt[2])) + 2*t*t*(start_pt[2] - 2*mid_pt[2] + end_pt[2])
#
#     stop = timeit.default_timer()
#     print stop - start
#
#     #ax.scatter(sample_pt[0], sample_pt[1], sample_pt[2])
#     ax.scatter(x_pt, y_pt, z_pt)
#     plt.show()
#
#
#
# def step_2(curr_pos, new_pos, step_height):
#     global leg1_q
#     #parabola function between 2 points
#     start = timeit.default_timer()
#
#     start_pt = np.array([curr_pos[0], curr_pos[1], curr_pos[2]])
#     end_pt = np.array([new_pos[0], new_pos[1], new_pos[2]])
#
#     #step_height = 5     #the height difference between steps, relative offset from start Z
#     t = np.linspace(0, 1, 10)
#
#     mid = (start_pt + end_pt) / float(2)
#     #mid = (curr_pos + new_pos) / float(2)
#
#     mid_pt = np.array([mid[0], mid[1], mid[2]+step_height])
#
#     x_pts = np.matrix([[curr_pos[0]], [mid_pt[0]], [new_pos[0]]])
#     y_pts = np.matrix([[curr_pos[1]], [mid_pt[1]], [new_pos[1]]])
#     z_pts = np.matrix([[curr_pos[2]], [mid_pt[2]], [new_pos[2]]])
#
#     A_1 = np.matrix([[2, -4, 2], [-3, 4, -1], [1, 0, 0]])
#
#     x_coeff = A_1 * x_pts
#     y_coeff = A_1 * y_pts
#     z_coeff = A_1 * z_pts
#
#     x = x_coeff.item(0)*t*t + x_coeff.item(1)*t + x_coeff.item(2)
#     y = y_coeff.item(0)*t*t + y_coeff.item(1)*t + y_coeff.item(2)
#     z = z_coeff.item(0)*t*t + z_coeff.item(1)*t + z_coeff.item(2)
#
#     stop = timeit.default_timer()
#     print "The Time:", stop - start
#
#     # main.
#     print main.leg1_q.get()
#
#     # map(main.leg1_q.put, x)
#
#     #Plot the trajectory of the
#         # fig = plt.figure()
#         # ax = fig.add_subplot(111, projection='3d')
#         # ax.set_xlim3d(0, 10)
#         # ax.set_ylim3d(0, 10)
#         # ax.set_zlim3d(0, 6)
#         # plt.axis([0, 5, 0, 5])
#         # plt.ion()
#         #
#         # for i in range(len(x)):
#         #     ax.scatter(x[i], y[i], z[i])
#         #     plt.pause(0.001)
#         #
#         # plt.pause(1)
#
#         # while True:
#         #     plt.pause(0.05)
#
#     return new_pos
#
#

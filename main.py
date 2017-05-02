from inverseKinematics import *
from piecewiseMotion import *
from motion import *
from servoControl import startSerial
from inverseKinematics import loadAnfisNetwork
import Queue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import timeit
import thread
from threading import Thread
from Blynk import *
from time import *

blynk_x_pos = 0
blynk_y_pos = 0

def main():
    print "Start Serial"
    startSerial()

    print "Open ANFIS"
    loadAnfisNetwork()
    print "ANFIS OPEN"

    print "Starting Blynk Connection"
    blynk_control_thread = Thread(target=blynk_controller, args=())
    blynk_control_thread.start()

    step_angle = 90
    step_length = 100
    step_height = 0
    step_precision = 250
    num_steps = 3

    while True:
        curr_pos = walk_dir(step_length, step_height, step_angle, num_steps, step_precision)
        homePos(curr_pos)
        time.sleep(2)
        step_angle += 45
        # print "Done Moving"



# walking gait, can walk in a direction and particular number of steps.
# The precision is the number incremental steps between the start and stop motions
def walk_dir(step_length, step_height, degrees, step_num, precision):
    #calculate the walking trajectory of one step
    # walking_trajectory = piecewiseMotion(step_length, step_height, degrees, precision)
    walking_trajectory_R = piecewiseMotion_3(step_length, step_height, degrees, -200, precision)
    walking_trajectory_L = piecewiseMotion_3(step_length, step_height, degrees - 180, -200, precision)

    # initialize the index of each leg, offset all of them
    FL_leg_index = 0
    FR_leg_index = precision / 4
    HL_leg_index = 2 * (precision / 4)
    HR_leg_index = 3 * (precision / 4)

    leg_index = [FL_leg_index, FR_leg_index, HL_leg_index, HR_leg_index]

    steps = 0

    home_pos = [0, 0, -200]
    start_step_precision = 150

    parabola_motion_2 = parabolaStep(home_pos, walking_trajectory_L[FR_leg_index], 75, -200, start_step_precision)
    parabola_motion_3 = parabolaStep(home_pos, walking_trajectory_R[HL_leg_index], 75, -200, start_step_precision)

    for index in range(0, start_step_precision):
        leg1 = inverseKinematics(home_pos)
        leg2 = inverseKinematics(parabola_motion_2[index])
        leg3 = inverseKinematics(parabola_motion_3[index])
        leg4 = inverseKinematics(home_pos)
        leg_1_servoval = angleToServoValue(leg1, 1)
        leg_2_servoval = angleToServoValue(leg2, 2)
        leg_3_servoval = angleToServoValue(leg3, 3)
        leg_4_servoval = angleToServoValue(leg4, 4)
        serialSend_All(leg_1_servoval, leg_2_servoval, leg_3_servoval, leg_4_servoval)


    # step a certain amount of times
    while(steps < step_num):

        print "Blynk: ", blynk_x_pos, blynk_y_pos
        # print "Front Left:", walking_trajectory[FL_leg_index]
        # print "Front Right:", walking_trajectory[FR_leg_index]
        # print "Hind Left:", walking_trajectory[HL_leg_index]
        # print "Hind Right:", walking_trajectory[HR_leg_index]

        move_to_pos(walking_trajectory_R, walking_trajectory_L, leg_index)

        leg_index = [x + 1 for x in leg_index]

        # if reached index limit, loop back and restart index count
        if leg_index[0] >= precision:
            leg_index[0] = 0
            steps += 1  # keep track of the number of steps taken

        if leg_index[1] >= precision:
            leg_index[1] = 0

        if leg_index[2] >= precision:
            leg_index[2] = 0

        if leg_index[3] >= precision:
            leg_index[3] = 0

    return [walking_trajectory_R[leg_index[0]], walking_trajectory_R[leg_index[1]], walking_trajectory_L[leg_index[2]], walking_trajectory_L[leg_index[3]]]




# parabola function between 2 points
def step_to(leg, curr_pos, new_pos, step_height):
    global leg1_q

    # start a timer for benchmarking purposes
    start = timeit.default_timer()

    # convert python array to numpy array to streamline math
    start_pt = np.array([curr_pos[0], curr_pos[1], curr_pos[2]])
    end_pt = np.array([new_pos[0], new_pos[1], new_pos[2]])

    # generate numpy array of numbers 0 to 1, to be used in parametric equations
    t = np.linspace(0, 1, 20)

    # determine the mean of the start and end points
    mid = (start_pt + end_pt) / float(2)

    # determine the actual mid point with the step_height factored in
    mid_pt = np.array([mid[0], mid[1], mid[2] + step_height])

    # create numpy matrix of all x, y z points
    x_pts = np.matrix([[curr_pos[0]], [mid_pt[0]], [new_pos[0]]])
    y_pts = np.matrix([[curr_pos[1]], [mid_pt[1]], [new_pos[1]]])
    z_pts = np.matrix([[curr_pos[2]], [mid_pt[2]], [new_pos[2]]])

    # generate the standard inverse matrix to solve parabolic constraints
    A_1 = np.matrix([[2, -4, 2], [-3, 4, -1], [1, 0, 0]])

    # solve all coefficients by multiplying inverse with points
    x_coeff = A_1 * x_pts
    y_coeff = A_1 * y_pts
    z_coeff = A_1 * z_pts

    # plug in solved coefficents to determine parametric equation for each axis
    x = x_coeff.item(0)*t*t + x_coeff.item(1)*t + x_coeff.item(2)
    y = y_coeff.item(0)*t*t + y_coeff.item(1)*t + y_coeff.item(2)
    z = z_coeff.item(0)*t*t + z_coeff.item(1)*t + z_coeff.item(2)

    pos = []

    # create matrix of all positions along trajectory
    for i in range(len(x)):
        pos.append([x[i], y[i], z[i]])

    # add each row of the matrix to the queue
    map(leg.put, pos)

    # stop the timer to for benchmarking purposes
    stop = timeit.default_timer()
    print "The Time:", stop - start

    # return the final position
    return new_pos


def add_leg1(first_pos, new_pos):
    #args first_pos and new_pos must be arrays

    curr_pos1 = step_to(leg1_q, first_pos, new_pos, 5)
    return


def blynk_controller():
    auth_token = "340c28ef62d94a998855b7c8d4b89651"

    blynk = Blynk(auth_token)

    while not blynk.app_status():
        print "Phone not connected"
        sleep(0.5)

    global blynk_height, blynk_x_pos, blynk_y_pos

    # create objects
    slider_height = Blynk(auth_token, pin="V0")
    joystick_pos = Blynk(auth_token, pin="V1")

    # get current status
    while (1):
        curr_height = slider_height.get_val()
        curr_pos = joystick_pos.get_val()
        blynk_height = int(curr_height[0])
        blynk_x_pos = -(512 - int(curr_pos[0]))
        blynk_y_pos = -(512 - int(curr_pos[1]))

        sleep(0.001)


def homePos(curr_pos):
    home_pos = [0, 0, -200]
    step_height = 75
    precision = 150

    leg_1_pos = curr_pos[0]
    leg_2_pos = curr_pos[1]
    leg_3_pos = curr_pos[2]
    leg_4_pos = curr_pos[3]


    if curr_pos[0][0] > 0:
        parabola_motion_1 = parabolaStep(leg_1_pos, home_pos, 0, -200, precision)
        for index in range(0, precision):
            leg1 = inverseKinematics(parabola_motion_1[index])
            leg2 = inverseKinematics(curr_pos[1])
            leg3 = inverseKinematics(curr_pos[2])
            leg4 = inverseKinematics(curr_pos[3])
            leg_1_servoval = angleToServoValue(leg1, 1)
            leg_2_servoval = angleToServoValue(leg2, 2)
            leg_3_servoval = angleToServoValue(leg3, 3)
            leg_4_servoval = angleToServoValue(leg4, 4)
            serialSend_All(leg_1_servoval, leg_2_servoval, leg_3_servoval, leg_4_servoval)
        curr_pos[0] = parabola_motion_1[index]

    elif curr_pos[0][0] < 0:
        parabola_motion_1 = parabolaStep(leg_1_pos, home_pos, step_height, -200, precision)

    if curr_pos[1][0] > 0:
        parabola_motion_2 = parabolaStep(leg_2_pos, home_pos, 0, -200, precision)
        for index in range(0, precision):
            leg1 = inverseKinematics(curr_pos[0])
            leg2 = inverseKinematics(parabola_motion_2[index])
            leg3 = inverseKinematics(curr_pos[2])
            leg4 = inverseKinematics(curr_pos[3])
            leg_1_servoval = angleToServoValue(leg1, 1)
            leg_2_servoval = angleToServoValue(leg2, 2)
            leg_3_servoval = angleToServoValue(leg3, 3)
            leg_4_servoval = angleToServoValue(leg4, 4)
            serialSend_All(leg_1_servoval, leg_2_servoval, leg_3_servoval, leg_4_servoval)
        curr_pos[1] = parabola_motion_2[index]

    elif curr_pos[1][0] < 0:
        parabola_motion_2 = parabolaStep(leg_2_pos, home_pos, step_height, -200, precision)

    if curr_pos[2][0] < 0:
        parabola_motion_3 = parabolaStep(leg_3_pos, home_pos, 0, -200, precision)
        for index in range(0, precision):
            leg1 = inverseKinematics(curr_pos[0])
            leg2 = inverseKinematics(curr_pos[1])
            leg3 = inverseKinematics(parabola_motion_3[index])
            leg4 = inverseKinematics(curr_pos[3])
            leg_1_servoval = angleToServoValue(leg1, 1)
            leg_2_servoval = angleToServoValue(leg2, 2)
            leg_3_servoval = angleToServoValue(leg3, 3)
            leg_4_servoval = angleToServoValue(leg4, 4)
            serialSend_All(leg_1_servoval, leg_2_servoval, leg_3_servoval, leg_4_servoval)
        curr_pos[2] = parabola_motion_3[index]

    elif curr_pos[2][0] > 0:
        parabola_motion_3 = parabolaStep(leg_3_pos, home_pos, step_height, -200, precision)

    if curr_pos[3][0] < 0:
        parabola_motion_4 = parabolaStep(leg_4_pos, home_pos, 0, -200, precision)
        for index in range(0, precision):
            leg1 = inverseKinematics(curr_pos[0])
            leg2 = inverseKinematics(curr_pos[1])
            leg3 = inverseKinematics(curr_pos[2])
            leg4 = inverseKinematics(parabola_motion_4[index])
            leg_1_servoval = angleToServoValue(leg1, 1)
            leg_2_servoval = angleToServoValue(leg2, 2)
            leg_3_servoval = angleToServoValue(leg3, 3)
            leg_4_servoval = angleToServoValue(leg4, 4)
            serialSend_All(leg_1_servoval, leg_2_servoval, leg_3_servoval, leg_4_servoval)
        curr_pos[3] = parabola_motion_4[index]

    elif curr_pos[3][0] > 0:
        parabola_motion_4 = parabolaStep(leg_4_pos, home_pos, step_height, -200, precision)


    for index in range(0, precision):
        if curr_pos[0][0] != 0:
            leg1 = inverseKinematics(parabola_motion_1[index])
        if curr_pos[1][0] != 0:
            leg2 = inverseKinematics(parabola_motion_2[index])
        if curr_pos[2][0] != 0:
            leg3 = inverseKinematics(parabola_motion_3[index])
        if curr_pos[3][0] != 0:
            leg4 = inverseKinematics(parabola_motion_4[index])

        leg_1_servoval = angleToServoValue(leg1, 1)
        leg_2_servoval = angleToServoValue(leg2, 2)
        leg_3_servoval = angleToServoValue(leg3, 3)
        leg_4_servoval = angleToServoValue(leg4, 4)
        serialSend_All(leg_1_servoval, leg_2_servoval, leg_3_servoval, leg_4_servoval)

if __name__ == "__main__":
    main()

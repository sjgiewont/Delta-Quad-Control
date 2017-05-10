import numpy as np
import Adafruit_BBIO.UART as UART
import serial
import time

UART.setup("UART1")
ser = serial.Serial(port="/dev/ttyO1", baudrate=115200)
ser.close()
ser.open()

if ser.isOpen():
    print "Serial is open!"

leg1 = np.array([16, 17, 19])
leg2 = np.array([0, 1, 2])
leg3 = np.array([28, 30, 31])
leg4 = np.array([12, 13, 14])

leg_1_center = np.array([1560, 1550, 1560])
leg_2_center = np.array([1560, 1705, 1720])
leg_3_center = np.array([1780, 1700, 1850])
leg_4_center = np.array([1500, 1590, 1680])


while True:
    cmd1 = "#0 P%d #1 P%d #2 P%d " % (leg_2_center[0], leg_2_center[1], leg_2_center[2])
    cmd2 = "#12 P%d #13 P%d #14 P%d" % (leg_4_center[0], leg_4_center[1], leg_4_center[2])
    cmd3 = "#16 P%d #17 P%d #19 P%d" % (leg_1_center[0], leg_1_center[1], leg_1_center[2])
    cmd4 = "#28 P%d #30 P%d #31 P%d \r" % (leg_3_center[0], leg_3_center[1], leg_3_center[2])

    final_cmd = " ".join((cmd1, cmd2, cmd3, cmd4))
    print "Final command: ", final_cmd
    ser.write(final_cmd)

    user_leg_sel = int(input("Choose a servo leg, 1, 2, 3, 4. Press 9 to EXIT: "))

    if user_leg_sel == 9:
        break

    user_servo_sel = int(input("Choose a servo index, 0, 1 or 2: "))
    user_input = int(input("Enter the step: "))

    if user_input > 100 or user_input < -100:
        print "Error: Enter smaller number"
        continue

    if user_leg_sel == 1:
        leg_1_center[user_servo_sel] = leg_1_center[user_servo_sel] + user_input
        # if user_input == 1:
        #     leg_1_center[user_servo_sel] = leg_1_center[user_servo_sel] + 1
        # elif user_input == 0:
        #     leg_1_center[user_servo_sel] = leg_1_center[user_servo_sel] - 1
        # else:
        #     print "ERROR: Wrong value entered!"
    elif user_leg_sel == 2:
        leg_2_center[user_servo_sel] = leg_2_center[user_servo_sel] + user_input
        # if user_input == 1:
        #     leg_2_center[user_servo_sel] = leg_2_center[user_servo_sel] + 1
        # elif user_input == 0:
        #     leg_2_center[user_servo_sel] = leg_2_center[user_servo_sel] - 1
        # else:
        #     print "ERROR: Wrong value entered!"
    elif user_leg_sel == 3:
        leg_3_center[user_servo_sel] = leg_3_center[user_servo_sel] + user_input
        # if user_input == 1:
        #     leg_3_center[user_servo_sel] = leg_3_center[user_servo_sel] + 1
        # elif user_input == 0:
        #     leg_3_center[user_servo_sel] = leg_3_center[user_servo_sel] - 1
        # else:
        #     print "ERROR: Wrong value entered!"
    elif user_leg_sel == 4:
        leg_4_center[user_servo_sel] = leg_4_center[user_servo_sel] + user_input
        # if user_input == 1:
        #     leg_4_center[user_servo_sel] = leg_4_center[user_servo_sel] + 1
        # elif user_input == 0:
        #     leg_4_center[user_servo_sel] = leg_4_center[user_servo_sel] - 1
        # else:
        #     print "ERROR: Wrong value entered!"
    else:
        print "ERROR: Wrong leg entered"


print "DONE"

print leg_1_center, leg_2_center, leg_3_center, leg_4_center

ser.close()

import numpy as np
import Adafruit_BBIO.UART as UART
import serial


def startSerial():
    global ser
    UART.setup("UART1")
    ser = serial.Serial(port="/dev/ttyO1", baudrate=115200)
    ser.close()
    ser.open()
    if ser.isOpen():
        print "Serial is open!"
        return ser


def angleToServoValue(thetas, leg_num):
    if leg_num == 1:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500), mapping(thetas[1],270,90,500,2500), mapping(thetas[2],270,90,500,2500)]))
    elif leg_num == 2:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500), mapping(thetas[1],270,90,500,2500), mapping(thetas[2],270,90,500,2500)]))
    elif leg_num == 3:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500), mapping(thetas[1],270,90,500,2500), mapping(thetas[2],270,90,500,2500)]))
    elif leg_num == 4:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500), mapping(thetas[1],270,90,500,2500), mapping(thetas[2],270,90,500,2500)]))
    else:
        return ValueError

    if servoValues[0] > 2400:
        servoValues[0] = 2400
    elif servoValues[0] < 600:
        servoValues[0] = 600

    if servoValues[1] > 2400:
        servoValues[1] = 2400
    elif servoValues[1] < 600:
        servoValues[1] = 600

    if servoValues[2] > 2400:
        servoValues[2] = 2400
    elif servoValues[2] < 600:
        servoValues[2] = 600

    return servoValues


def mapping(value, fromLow, fromHigh, toLow, toHigh):
    return (((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow


def serialSend_All(leg_1_thetas, leg_2_thetas, leg_3_thetas, leg_4_thetas):
    cmd1 = "#1 P%d #2 P%d #3 P%d " % (leg_1_thetas[0], leg_1_thetas[1], leg_1_thetas[2])
    cmd2 = "#12 P%d #13 P%d #14 P%d" % (leg_2_thetas[0], leg_2_thetas[1], leg_2_thetas[2])
    cmd3 = "#16 P%d #17 P%d #18 P%d" % (leg_3_thetas[0], leg_3_thetas[1], leg_3_thetas[2])
    cmd4 = "#28 P%d #29 P%d #30 P%d" % (leg_4_thetas[0], leg_4_thetas[1], leg_4_thetas[2])

    final_cmd = " ".join((cmd1, cmd2, cmd3, cmd4))
    print final_cmd
    ser.write(final_cmd)
    return

def serialSend_one(servo_values, leg_value):
    leg1 = np.array([1, 2, 3])
    leg2 = np.array([4, 5, 6])
    leg3 = np.array([7, 8, 9])
    leg4 = np.array([10, 11, 12])

    if leg_value == 1:
        leg = leg1
    elif leg_value == 2:
        leg = leg2
    elif leg_value == 3:
        leg = leg3
    elif leg_value == 4:
        leg = leg4
    else:
        return ValueError

    final_cmd = "#%d P%d #%d P%d #%d P%d \r" % (leg[0], servo_values[0], leg[1], servo_values[1], leg[2], servo_values[2])
    # print final_cmd

    ser.write(final_cmd)

    return 0



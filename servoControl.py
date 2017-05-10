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
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) + 60, mapping(thetas[1],270,90,500,2500) + 50, mapping(thetas[2],270,90,500,2500) + 60]))
    elif leg_num == 2:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) + 60, mapping(thetas[1],270,90,500,2500) + 205, mapping(thetas[2],270,90,500,2500) + 220]))
    elif leg_num == 3:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) + 280, mapping(thetas[1],270,90,500,2500) + 200, mapping(thetas[2],270,90,500,2500) + 350]))
    elif leg_num == 4:
        servoValues = np.around(np.array([mapping(thetas[0],270,90,500,2500) + 330, mapping(thetas[1],270,90,500,2500) + 90, mapping(thetas[2],270,90,500,2500) + 180]))
    else:
        return ValueError

    if servoValues[0] > 2400:
        print "ERROR: SERVO 0 OUT OF RANGE - HIGH", servoValues[0], leg_num
        servoValues[0] = 2400
    elif servoValues[0] < 600:
        print "ERROR: SERVO 0 OUT OF RANGE - LOW", servoValues[0], leg_num
        servoValues[0] = 600

    if servoValues[1] > 2400:
        print "ERROR: SERVO 1 OUT OF RANGE - HIGH", servoValues[1], leg_num
        servoValues[1] = 2400
    elif servoValues[1] < 600:
        print "ERROR: SERVO 1 OUT OF RANGE - LOW", servoValues[1], leg_num
        servoValues[1] = 600

    if servoValues[2] > 2400:
        print "ERROR: SERVO 2 OUT OF RANGE - HIGH", servoValues[2], leg_num
        servoValues[2] = 2400
    elif servoValues[2] < 600:
        print "ERROR: SERVO 2 OUT OF RANGE - LOW", servoValues[2], leg_num
        servoValues[2] = 600

    return servoValues


def mapping(value, fromLow, fromHigh, toLow, toHigh):
    return (((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow


def serialSend_All(leg_1_thetas, leg_2_thetas, leg_3_thetas, leg_4_thetas):
    cmd1 = "#0 P%d #1 P%d #2 P%d " % (leg_2_thetas[0], leg_2_thetas[1], leg_2_thetas[2])
    cmd2 = "#12 P%d #13 P%d #14 P%d" % (leg_4_thetas[0], leg_4_thetas[1], leg_4_thetas[2])
    cmd3 = "#16 P%d #17 P%d #19 P%d" % (leg_1_thetas[0], leg_1_thetas[1], leg_1_thetas[2])
    cmd4 = "#28 P%d #30 P%d #31 P%d \r" % (leg_3_thetas[0], leg_3_thetas[1], leg_3_thetas[2])

    final_cmd = " ".join((cmd1, cmd2, cmd3, cmd4))
    # print final_cmd
    ser.write(final_cmd)


def serialSendServoVals(leg_1_thetas, leg_2_thetas, leg_3_thetas, leg_4_thetas):
    leg_1_servo = angleToServoValue(leg_1_thetas, 1)
    leg_2_servo = angleToServoValue(leg_2_thetas, 2)
    leg_3_servo = angleToServoValue(leg_3_thetas, 3)
    leg_4_servo = angleToServoValue(leg_4_thetas, 4)

    cmd1 = "#0 P%d #1 P%d #2 P%d " % (leg_2_servo[0], leg_2_servo[1], leg_2_servo[2])
    cmd2 = "#12 P%d #13 P%d #14 P%d" % (leg_4_servo[0], leg_4_servo[1], leg_4_servo[2])
    cmd3 = "#16 P%d #17 P%d #19 P%d" % (leg_1_servo[0], leg_1_servo[1], leg_1_servo[2])
    cmd4 = "#28 P%d #30 P%d #31 P%d \r" % (leg_3_servo[0], leg_3_servo[1], leg_3_servo[2])

    final_cmd = " ".join((cmd1, cmd2, cmd3, cmd4))
    print final_cmd
    ser.write(final_cmd)


def serialSend(ser, serial_string):
    print 'hi'
    print serial_string
    ser.write(serial_string)
    print 'bye'
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



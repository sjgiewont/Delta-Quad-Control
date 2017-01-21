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

leg1 = np.array([1, 2, 3])
leg2 = np.array([4, 5, 6])
leg3 = np.array([7, 8, 9])
leg4 = np.array([10, 11, 12])

# choose which leg here
leg_value = 1

if leg_value == 1:
    leg = leg1
elif leg_value == 2:
    leg = leg2
elif leg_value == 3:
    leg = leg3
elif leg_value == 4:
    leg = leg4
else:
    print "Error"

servo_values = np.array([1550, 1550, 1550])

final_cmd = "#%d P%d #%d P%d #%d P%d \r" % (leg[0], servo_values[0], leg[1], servo_values[1], leg[2], servo_values[2])
ser.write(final_cmd)

time.sleep(1)

print "DONE"

ser.close()

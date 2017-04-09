from Blynk import *
from time import sleep


auth_token = "340c28ef62d94a998855b7c8d4b89651"

blynk = Blynk(auth_token)

while(not blynk.app_status()):
    print "Phone not connected"
    sleep(0.5)

# create objects
slider_height = Blynk(auth_token, pin="V0")
joystick_pos = Blynk(auth_token, pin="V1")

# get current status
while(1):
    curr_height = slider_height.get_val()
    curr_pos = joystick_pos.get_val()
    height = int(curr_height[0])
    x_pos = -(512 - int(curr_pos[0]))
    y_pos = -(512 - int(curr_pos[1]))

    print height, x_pos, y_pos
    sleep(0.001)



import timeit
import sys
from socket import *
from threading import Thread
from Queue import Queue
from servoControl import *
import numpy as np

MAX_LENGTH = 96

def recieve_socket_commands(clientsocket, my_queue):
    angle = 4.5
    buf = []

    try:
        while 1:
            buf = clientsocket.recv(MAX_LENGTH)
            buf_string = buf.decode()
            buf_array = buf_string.split(",", 11)
            if len(buf_array) != 12:
                buf = []
                print "ERROR: Too many/not enough arguments received"
                continue
            else:
                my_queue.put(buf_array)
    except:
        serversocket.close()
        return

    # receive the commands here
    # buf = clientsocket.recv(MAX_LENGTH)
    # buf_string = buf.decode()
    # # print(buf_string)
    # buf_array = buf_string.split(",", 11)
    # # print(buf_array)
    # my_queue.put(buf_array)

    # print(int(buf_array[1]) + int(buf_array[0]))
    # if buf == '':
    #     return #client terminated connection
    # if buf:
    #     print(float(buf_array[0]), float(buf_array[1]), float(buf_array[2]))


# setup a socket that will recieve commands from Python2 code
serversocket = socket(AF_INET, SOCK_STREAM)

serversocket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)

# define port and host address
PORT = 12345
HOST = '129.21.90.103'

serversocket.bind((HOST, PORT))
serversocket.listen(10)


print('Start Accept connections')
# accept connections from outside
(clientsocket, address) = serversocket.accept()
print('Accept connections')

my_queue = Queue(maxsize=0)

socket_read_thread = Thread(target=recieve_socket_commands, args=(clientsocket, my_queue))
socket_read_thread.start()
# recieve_socket_commands(clientsocket)

try:
    while 1:
        buf_array = my_queue.get()
        # print buf_array
        # print(float(buf_array[0]), float(buf_array[1]), float(buf_array[2]))
        leg_1_servo = angleToServoValue([float(buf_array[0]), float(buf_array[1]), float(buf_array[2])], 1)
        leg_2_servo = angleToServoValue([float(buf_array[3]), float(buf_array[4]), float(buf_array[5])], 2)
        leg_3_servo = angleToServoValue([float(buf_array[6]), float(buf_array[7]), float(buf_array[8])], 3)
        leg_4_servo = angleToServoValue([float(buf_array[9]), float(buf_array[10]), float(buf_array[11])], 4)
        # serialSend_All(leg_1_servo, leg_2_servo, leg_3_servo, leg_4_servo)
        # print leg_1_servo, leg_2_servo, leg_3_servo, leg_4_servo

except:
    my_queue.join()
    socket_read_thread.join()
    serversocket.close()
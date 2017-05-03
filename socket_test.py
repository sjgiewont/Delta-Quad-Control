import timeit
import sys
from socket import *
from threading import Thread
from Queue import Queue
import numpy as np

MAX_LENGTH = 95

def recieve_socket_commands(clientsocket, my_queue):
  angle = 4.5
  while 1:
    # receive the commands here
    buf = clientsocket.recv(MAX_LENGTH)
    buf_string = buf.decode()
    # print(buf_string)
    buf_array = buf_string.split(",", 11)
    # print(buf_array)
    my_queue.put(buf_array)

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
        print buf_array

        # if buf == '':
        #     return  # client terminated connection
        # if buf:
        #     print(float(buf_array[0]), float(buf_array[1]), float(buf_array[2]))

        print(float(buf_array[0]), float(buf_array[1]), float(buf_array[2]))
except:
    my_queue.join()
    socket_read_thread.join()
    serversocket.close()
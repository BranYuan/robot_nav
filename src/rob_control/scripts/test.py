import socket
import time
from server_cmd import *
print GOAL_POSITION
HOST = '127.0.0.1'
PORT = 50008
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.sendall('*-0.823,-1,-0.07#')
"""if 1:# try:
    s.connect((HOST, PORT))
# except:
    pass
# while 1:
try:
    s.sendall('*-0.823,-1,-0.07#')
    data = s.recv(1024)
    print 'Sended : ', repr(data)
except:
    print ('err')"""
time.sleep(2)

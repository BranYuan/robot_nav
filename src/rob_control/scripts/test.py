import socket
import time
from server_cmd import *
print GOAL_POSITION
HOST = '127.0.0.1'
PORT = 50008
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
if 1:# try:
    s.connect((HOST, PORT))
# except:
    pass
while 1:
    try:
        s.sendall('*0.0,-0.99398,0.18418#')
        data = s.recv(1024)
        print 'Sended : ', repr(data)
    except:
        print ('err')
    time.sleep(2)

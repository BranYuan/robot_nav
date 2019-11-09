#!/usr/bin/env python
# __*__coding:utf-8__*__

import threading
import socket
import os
import re


def arm_init():
    # socket处理线程
    socket_thread = threading.Thread(target=socket_loop, name='SocketThread')
    socket_thread.start()
    print('socket')
    socket_thread.join()

def socket_loop():
    server_main()


def kill_process(port = 50008):
    pass


def server_main():
    host = '0.0.0.0'
    port = 50008
    kill_process()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((host, port))
    s.listen(1)
    while True:
        # 等待CLIENT连接
        try:
            print('Waitting for socket client')
            conn, addr = s.accept()
            if addr:
                break
        except:
            pass
    goal_err = False
    while True:
        ## 主循环
        try:
            ## 接收并GOAL_POSITION = [0.0, -0.99398, 0.18418] # 梨目标所在位置
            data = conn.recv(1024)
            print(data)
            if data:
                conn.sendall('*received#')
        except:
            # 如果出错重新监听

            print 'err'
            try:
                kill_process()
                conn.close()
                s.close()
                conn = None
                s = None
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.bind((host, port))
                s.listen(1)
                conn, addr = s.accept()
            except:
                pass


if __name__ == '__main__':
    arm_init()
#!/usr/bin/env python
# __*__coding:utf-8__*__
import socket
import os
import re
import time
import rospy

GOAL_POSITION = [0.0, -0.99398, 0.18418] # 梨目标所在位置
GOAL_FLAG = -1 # 是否有目标传入标志,1:目标传入，2:机械手抓取成功，3:抓取失败,0:等待机械手准备好

def kill_process(port = 50008):
    ret = os.popen('netstat -nao|findstr' + str(port))
    str_list = ret.read().decode('gbk')
    ret_list = re.split('', str_list)
    try:
        process_pid = list(ret_list[0].split())[-1]
        os.popen('taskkill /pid ' + str(process_pid) + ' /F')
    except:
        return True
def server_main():
    ## socket线程主循环，接收来自视觉的目标请求命令
    global GOAL_POSITION
    global GOAL_FLAG
    host = '0.0.0.0'
    port = 50008
    kill_process()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((host, port))
    s.listen(10)
    while not rospy.is_shutdown():
        # 等待CLIENT连接
        try:
            print('Waitting for socket client')
            conn, addr = s.accept()
            if addr:
                break
        except:
            pass
    goal_err = False

    while not GOAL_FLAG == 0:
        ## 等待机械手初始化完成
        print('Waitting for arm init')
        print(str(GOAL_FLAG))
        time.sleep(2)
    while not rospy.is_shutdown():
        ## 主循环
        try:
            ## 接收并GOAL_POSITION = [0.0, -0.99398, 0.18418] # 梨目标所在位置
            data = conn.recv(1024)
            print(data)
            s_head = data.rfind('*')
            s_end = data.rfind('#')
            if s_end > s_head:
                data = data[s_head+1:s_end]
                data = data.split(',')
            else:
                data = None

            if data:
                print(data)
                for i in range(len(data)):
                    ## data转换成float类型
                    try:
                        data[i] = float(data[i])
                    except:
                        goal_err = True
                        break
                if not goal_err:
                    # 返回收到合法data
                    conn.sendall('*received#')
                    GOAL_POSITION = copy.deepcopy(data)
                    GOAL_FLAG = 1
                    while GOAL_FLAG == 1:
                        # 等待机械手执行目标结果
                        time.sleep(2)
                    if GOAL_FLAG == 2:
                        # 返回目标抓取结果
                        conn.sendall('*get_goal#')
                    elif GOAL_FLAG == 3:
                        conn.sendall('*failed#')
                else:
                    # 返回受到不合法命令
                    conn.sendall('*err#')
                    goal_err = True
        except:
            ## 如果出错重新监听
            kill_process()
            print 'err'
            try:
                s = None
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.bind((host, port))
                s.listen(1)
                conn, addr = s.accept()
                goal_err = True
            except:
                kill_process()
if __name__ == '__main__':
    server_main()

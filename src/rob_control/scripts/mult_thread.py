import threading
import time

class p:
    def __init__(self):
        pass

p = p()

def loop1(p):
    while 1:
        print 'loop1'
        time.sleep(0.5)

def loop2():
    while 1:
        print 'loop2'
        time.sleep(1)

def main_init():
    t1 = threading.Thread(target=loop1,args=(p,))
    t2 = threading.Thread(target=loop2)
    t1.start()
    t2.start()

if __name__ == '__main__':
    main_init()


#!/usr/bin/python
# coding:utf-8

import serial
import time


class SerialCmd(object):

    def __init__(self,port = None):
        if not port:
            self.port = '/dev/ttyS1'
        else:
            self.port = port
        self.bauterate = 9600
        self.bytesize = 8
        self.parity = 'N'
        self.stopbits = 1
        self.timeout = 0.5
        self.__portValid = False
        self.write_timeout = 0.2
        self.serial = serial.Serial(port=self.port, baudrate=self.bauterate, bytesize=self.bytesize,
                                    parity=self.parity, stopbits=self.stopbits, timeout=self.timeout,
                                    write_timeout=self.write_timeout)
        if self.serial.isOpen():
            self.__portValid = True

    def open_port(self, port=None):
        if port is not None:
            if port != self.port:
                self.port = port
                self.serial.port = port
            self.serial.open()
            self.__portValid = self.serial.isOpen()
            return self.__portValid
            print(self.serial.port)
        else:
            if not self.serial.isOpen():
                self.serial.open()
                self.__portValid = True
            print(self.serial.port)
            return self.__portValid

    def set_port(self):
        self.serial = serial.Serial(port=self.port, baudrate=self.bauterate, bytesize=self.bytesize,
                                    parity=self.parity, stopbits=self.stopbits, timeout=self.timeout,
                                    write_timeout=self.write_timeout)
        self.__portValid = self.serial.isOpen()
        return self.__portValid

    # 本方法仅支持发送全整数list,全string型list,int型，string型命令
    def send_cmd(self, cmd_value):
        # 命令类型标志，高四位为list 或 dict类型标志，list取1，dict取2，其余取0，低四位为list 或 dict内元素标志，int取1，string取2
        cmd_type = 0x00
        sum_int_type = 0
        sum_str_type = 0
        if isinstance(cmd_value, list):
            # 若列表里不全是整数，则返回False
            for i in range(cmd_value.__len__()):
                if isinstance(cmd_value[i], int):
                    sum_int_type += 1
                elif isinstance(cmd_value, str):
                    sum_str_type += 1
            if sum_int_type == cmd_value.__len__():
                self.serial.write(cmd_value)
                return True
            elif sum_str_type == cmd_value.__len__():
                pass
            else:
                return False

        elif isinstance(cmd_value, dict):
            pass
        elif isinstance(cmd_value, int):
            self.serial.write(cmd_value)
        elif isinstance(cmd_value, str):
            print('data type of cmd_value is string')
            for i in cmd_value:
                self.serial.write(i)
            return True
        else:
            return False

    def convert_hex(self, string):
        res = []
        result = []
        for item in string:
            res.append(item)
        for i in res:
            result.append(hex(i))
        return result

    def receive_info_str(self):
        response = ''
        if self.serial.in_waiting:
            response = self.serial.read_until('#')
            print(response)
            # response = self.convert_hex(response)
        return response


if __name__ == '__main__':
    print('start:')
    ser = SerialCmd()
    cmd = [0xA, 0x12, 0x26]
    while(1):
        print(ser.send_cmd(cmd))
        time.sleep(1)



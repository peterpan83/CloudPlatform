'''
@  author Yanqun Pan.
@  09/14 2018
@ State Key Laboratory of Esturine and Coastal Research,East China Normal University
'''


DEFAULT_PAN_SPEED = 10 # 0.15度/speed
DEFAULT_TILT_SPEED = 2

import abc
import serial
from serial import SerialException
import time

class Cprotocol:
    '''
    father class of the protocol for cloud platform
    '''
    m_SendBuf = []
    m_ulAddress = 1
    pan_speed = DEFAULT_PAN_SPEED
    def __init__(self,pPort,pLen):
        self.pPort = pPort
        self.pLen = pLen
        self.m_SendBuf = [0 for i in range(pLen)]

    @abc.abstractclassmethod
    def left(self,secs):
        pass

    @abc.abstractclassmethod
    def right(self,secs):
        pass

    @abc.abstractclassmethod
    def stop(self):
        pass

    @abc.abstractclassmethod
    def getHorizonLoc(self):
        pass

    @abc.abstractclassmethod
    def getVerticalLoc(self):
        pass

    @abc.abstractclassmethod
    def send(self):
        pass

    @abc.abstractclassmethod
    def send_get(self):
        pass

    @abc.abstractclassmethod
    def resetHorizon(self):
        pass

    def mereset(self):
        for i in range(len(self.m_SendBuf)):
            self.m_SendBuf[i] = 0

    @abc.abstractclassmethod
    def goto(self,degree):
        pass

class CprotocolD(Cprotocol):
    '''
    implementation of the pelco-D protocol
    '''
    def __init__(self,pPort,pLen):
        Cprotocol.__init__(self,pPort,pLen)

    def send(self):
        self.m_SendBuf[1] = self.m_ulAddress
        self.m_SendBuf[0] = 0XFF
        self.m_SendBuf[6] = self.m_SendBuf[1]
        for i in range(2,6):
            self.m_SendBuf[6] += self.m_SendBuf[i]
        self.pPort.write(self.m_SendBuf)

    def send_get(self):
        self.m_SendBuf[1] = self.m_ulAddress
        self.m_SendBuf[0] = 0XFF
        self.m_SendBuf[6] = self.m_SendBuf[1]
        for i in range(2,6):
            self.m_SendBuf[6] += self.m_SendBuf[i]
        self.pPort.write(self.m_SendBuf)
        response = self.pPort.readall()
        # return self.convert_hex(response)
        return response

    def convert_hex(self, string):
        res = []
        result = []
        for item in string:
            res.append(item)
        for i in res:
            result.append(hex(i))
        return result

    def left(self,secs=0):
        self.mereset()
        self.m_SendBuf[3] = 4
        self.m_SendBuf[4] = self.pan_speed
        self.send()
        time.sleep(secs)
        self.mereset()
        self.send()


    def right(self,secs=0):
        self.mereset()
        self.m_SendBuf[3] = 2
        self.m_SendBuf[4] = self.pan_speed
        self.send()
        # print(DEFAULT_PAN_SPEED)
        time.sleep(secs)
        self.mereset()
        self.send()

    def getVerticalLoc(self):
        self.mereset()
        self.m_SendBuf[3] = 0x53
        response = self.send_get()
        return response

    def getHorizonLoc(self):
        self.mereset()
        self.m_SendBuf[3] = 0x51
        response = self.send_get()
        return response

    def resetHorizon(self):
        self.mereset()
        self.m_SendBuf[2] = 0x3F
        self.m_SendBuf[3] = 0x4b
        self.m_SendBuf[4] = 0x00
        self.m_SendBuf[5] = 0x0b
        self.send()

    def stop(self):
        self.mereset()
        self.send()

    def goto(self,degree):
        '''协议有问题'''
        self.mereset()
        self.m_SendBuf[2] = 0x3F
        self.m_SendBuf[3] = 0x4b
        # self.m_SendBuf[4] = int('0x'+str(hex(int(170 / 0.0879))).split('x')[1][:-2],16)
        # self.m_SendBuf[5] = int('0x'+str(hex(int(170 / 0.0879))).split('x')[1][-2:],16)

        self.m_SendBuf[4] = 0x03
        self.m_SendBuf[5] = 0xff
        self.send()




if __name__ == '__main__':

    port ,baudrate,bytesize, parity, stopbits, timeout = 'COM3',9600,8,'E',1,0.1
    try:
        pPort = serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits,timeout=timeout)
    except SerialException as se:
            print ('open serial failed.%s'%(se))

    protocolD = CprotocolD(pPort,pLen=7)
    # protocolD.resetHorizon()
    # time.sleep(10)
    protocolD.left(10)

# -*- coding:utf-8 -*-
'''
@  author Yanqun Pan.
@  09/14 2018
@ State Key Laboratory of Esturine and Coastal Research,East China Normal University
'''
import time
from datetime import datetime
import datetime as dt
from Cprotocol import *
import Cprotocol
import numpy as np
import math,os
from apscheduler.schedulers.blocking import BlockingScheduler
import argparse
from enum import Enum
import sys
import configparser
from tkinter import *

scheduler = None
INVALID = -999

def calTimeAngel(longitude,longitude_timezone,time):
    '''
    计算时角
    :param longitude: 当地经度
    :param longitude_timezone: 当地时区所在经度，如北京时间120度
    :param time: 当地时区时间
    :return: 时角（度）
    '''
    #计算真太阳时
    # temp = time + dt.timedelta(minutes=(longitude_timezone-longitude)*4)
    temp = time + dt.timedelta(minutes=(longitude - longitude_timezone) * 4)
    realSolarTime = temp.hour+temp.minute/60.0+temp.second/3600.0
    # print(realSolarTime)
    timeAngle = (realSolarTime-12)*15
    return timeAngle

def calDeclination(month,day):
    '''
    计算太阳赤纬（度）
    :param month: 月份
    :param day: 天
    :return: 太阳赤纬（度）
    '''
    delta = datetime(2001, month, day)-datetime(2001, 1, 1)
    days = delta.days+1
    b = 2*np.pi*(days-1)/365.0
    declination = 0.006918-0.399912*np.cos(b)+0.070257*np.sin(b)-\
                  0.006758*np.cos(2*b)+0.000907*np.sin(2*b)-\
                  0.002697*np.cos(3*b)+0.00148*np.sin((3*b))

    return math.degrees(declination)


def calSolarElevationAzmuth(latitude,timeangle,declination):
    '''
    计算太阳高度和方位角
    :param latitude: 地理纬度
    :param timeangle: 时角
    :param declination: 太阳赤纬
    :return: 太阳高度角（度）和 方位角（度），正南方为0度，以东为负，以西为正
    '''
    sinHs = np.sin(latitude/180*np.pi)*np.sin(declination/180*np.pi)+\
           np.cos(latitude/180*np.pi)*np.cos(declination/180*np.pi)*np.cos(timeangle/180*np.pi)

    cosHs = np.cos(np.arcsin(sinHs))

    cosAs = (sinHs*np.sin(latitude/180*np.pi)-np.sin(declination/180*np.pi))/(cosHs*np.cos(latitude/180*np.pi))
    # print(cosAs)

    return np.arcsin(sinHs)*180/np.pi,np.arccos(cosAs)*180/np.pi


class Eaction(Enum):
    '''
    动作枚举
    '''
    left = 1
    right = 2
    up = 3
    down =4
    leftup = 5
    leftdown = 6
    rightup = 7
    rightdown = 8
    reset = 9

class CloudPlatform():
    def __init__(self,longitude,latitude,AzimuthSensorIni,AzimuthDelta,AzimuthSensorMin=0,AzimuthSensorMax=360,port='COM5',baudrate=9600,TIMEZONE='E08'):
        '''

        :param longitude: location of the platform deployed
        :param latitude:location of the platform deployed
        :param TIMEZONE: default East 8+, Beijing Time
        '''
        port, baudrate, bytesize, parity, stopbits, timeout = port, baudrate, 8, 'E', 1, 0.1
        try:
            self.pPort = serial.Serial(port=port, baudrate=baudrate, bytesize=bytesize, parity=parity, stopbits=stopbits,
                                  timeout=timeout)
        except SerialException as se:
            print('open serial[%s] failed.%s' % (port,se))
            sys.exit()

        self.AzimuthSensorIni = AzimuthSensorIni
        self.AzimuthDelta = AzimuthDelta

        self.protocolD = CprotocolD(self.pPort, pLen=7)
        # self.protocolD = None
        # self.pProtocol = protocolD

        self.currentHorizon = self.AzimuthSensorIni
        self.longitude,self.latitude = longitude,latitude
        self.AzimuthSensorMin = AzimuthSensorMin
        self.AzimuthSensorMax = AzimuthSensorMax
        self.scheduler = None
        self.pan_speed = Cprotocol.DEFAULT_PAN_SPEED
        print('init')


    def reset(self):
        '''
        让云台转到初始位置
        :return:
        '''
        self.pan_speed = Cprotocol.DEFAULT_PAN_SPEED
        if self.currentHorizon-self.AzimuthSensorIni >= 180:
            self.pan_speed = 30
            # 先左转180度,为了防止总是往一个方向转，线发生线缠绕的情况
            secs = 180 / (self.pan_speed * Cprotocol.PAN_SPEED_DEGREE)
            self.protocolD.left(secs,self.pan_speed)
            # print('左转了%d秒'%(secs))
        self.protocolD.resetHorizon()
        self.currentHorizon = self.AzimuthSensorIni
        print('云台已复位: %s'%(datetime.now()))

    def init(self):
        '''
        首先让云台转到初始位置，然后让传感器转到正北方向
        :return:
        '''
        self.reset()
        self.pan_speed = Cprotocol.DEFAULT_PAN_SPEED
        if abs(self.AzimuthSensorIni) > 10:
            self.pan_speed = 30
        secs = abs(self.AzimuthSensorIni)/(self.pan_speed*Cprotocol.PAN_SPEED_DEGREE)
        print("%s" % (datetime.now()))
        print('当前HyperSAS方位：%.2f,按当前速度%.3f/s,至正北需%s转%.3f秒。' % (self.AzimuthSensorIni, self.pan_speed*Cprotocol.PAN_SPEED_DEGREE, "左" if self.AzimuthSensorIni>0 else "右",secs))
        if self.AzimuthSensorIni > 0:
            self.protocolD.left(secs,self.pan_speed)
        else:
            self.protocolD.right(secs,self.pan_speed)
        self.currentHorizon = 0
        print("HyperSAS传感器已旋转至正北方向，%s"%(datetime.now()))

    def getIdealAzimuth(self,sa):
        '''
        :param sa:太阳方位角，以正北为基准
        :return: HySAS 理想角度, 不在范围内，返回-999
        '''
        AzimuthSensor = sa - self.AzimuthDelta
        if AzimuthSensor < self.AzimuthSensorMin:
            AzimuthSensor = sa + self.AzimuthDelta
            if AzimuthSensor > self.AzimuthSensorMax:
                return INVALID
        return AzimuthSensor


    def first_adjust(self):
        '''
        init后立即调整
        :return:
        '''
        self.pan_speed = Cprotocol.DEFAULT_PAN_SPEED
        now = datetime.now()
        timeangle = calTimeAngel(self.longitude, 120, now)
        # print(timeangle)
        dec = calDeclination(now.month, now.day)
        # print(dec)
        se, sa = calSolarElevationAzmuth(self.latitude, timeangle, dec)
        # print(sa)
        # 以正北为基准
        if timeangle > 0:
            sa = 180 + sa  ## 当前太阳的方位角
        else:
            sa = 180 - sa
        print(now)
        AzimuthSensor = self.getIdealAzimuth(sa=sa)
        if AzimuthSensor == INVALID:
            print("当前HyperSAS方位：%.2f,太阳方位：%.2f,HyperSAS理想方位%.2f不在[%.2f,%.2f]内，无法测量。"%(self.currentHorizon,
                                                                                    sa,AzimuthSensor,self.AzimuthSensorMin,self.AzimuthSensorMax))
            return
        else:
            if AzimuthSensor > 10:
                self.pan_speed = 30
            else:
                self.pan_speed = Cprotocol.DEFAULT_PAN_SPEED

            secs = AzimuthSensor / (self.pan_speed * Cprotocol.PAN_SPEED_DEGREE)  ##当前平台的方位角与太阳的方位角之差，在当前速度下所需时间

            print('当前HyperSAS方位：%.2f,太阳方位：%.2f,角度差：%.2f,按当前速度%.3f/s,需右转%.3f秒。'
                  % (self.currentHorizon, sa, AzimuthSensor, self.pan_speed * Cprotocol.PAN_SPEED_DEGREE, secs))
            self.protocolD.right(secs,pan_speed=self.pan_speed)
            self.currentHorizon = AzimuthSensor


    def auto(self):
        self.pan_speed = Cprotocol.DEFAULT_PAN_SPEED
        now = datetime.now()
        timeangle = calTimeAngel(self.longitude, 120, now)
        # print(timeangle)
        dec = calDeclination(now.month, now.day)
        # print(dec)
        se,sa = calSolarElevationAzmuth(self.latitude, timeangle, dec)
        # print(sa)

        #以正北为基准
        if timeangle >0:
            sa = 180 + sa ## 当前太阳的方位角
        else:
            sa = 180 - sa

        # AzimuthSensor = 360 - np.abs(self.AzimuthSensorIni -(sa-self.AzimuthDelta))
        AzimuthSensor = sa - self.AzimuthDelta
        print("\n %s"%(now))
        AzimuthSensor = self.getIdealAzimuth(sa=sa)
        if AzimuthSensor == INVALID:
            print("当前HyperSAS方位：%.2f,太阳方位：%.2f,HyperSAS理想方位%.2f不在[%.2f,%.2f]内，无法测量。" % (self.currentHorizon,sa, AzimuthSensor,
                                                                                        self.AzimuthSensorMin,
                                                                                        self.AzimuthSensorMax))
            return

        delta = AzimuthSensor-self.currentHorizon
        if abs(delta) > 10:
            self.pan_speed = 30

        secs = delta / (self.pan_speed * Cprotocol.PAN_SPEED_DEGREE)  ##当前平台的方位角与太阳的方位角之差，在当前速度下所需时间

        print('当前HyperSAS方位：%.2f,太阳方位：%.2f,理想方位：%.2f,角度差：%.2f,按当前速度%.3f/s,需%s转%.3f秒。'
              % (self.currentHorizon, sa,AzimuthSensor, delta, self.pan_speed*Cprotocol.PAN_SPEED_DEGREE, "左" if delta<0 else "右",secs))
        if delta <0: #左转
            self.protocolD.left(secs,pan_speed=self.pan_speed)
        else:
            self.protocolD.right(secs,pan_speed=self.pan_speed)

        self.currentHorizon = AzimuthSensor


    def setRuntime(self,start,end):
        self.starthour = int(start.split(':')[0])
        self.endhour = int(end.split(':')[0])

    def do(self,action):
        try:
            if action == Eaction.left.value:
                self.protocolD.left()
            if action == Eaction.right.value:
                self.protocolD.right()
        except KeyboardInterrupt:
            self.protocolD.stop()
        finally:
            self.closePort()

        if action == Eaction.reset.value:
            self.protocolD.resetHorizon()

    def closePort(self):
        if self.pPort.isOpen():
            self.pPort.close()

    def isRuning(self):
        response = self.protocolD.getHorizonLoc()
        print(str(response))
        if response==None or len(response)<4:
            return False
        return True


def autoRun(pCloudPlatform):
    scheduler = BlockingScheduler()
    print('开始时间%d点，结束时间%d点'%(pCloudPlatform.starthour,pCloudPlatform.endhour))
    scheduler.add_job(pCloudPlatform.auto, 'cron',minute='*/10', hour='%d-%d'%(pCloudPlatform.starthour,pCloudPlatform.endhour))
    scheduler.add_job(pCloudPlatform.init, 'cron',hour='1',minute='5')
    print('Press Ctrl+{0} to exit'.format( 'C' if os.name == 'nt' else 'Break'))
    try:
        scheduler.start()
    except (KeyboardInterrupt,SystemExit):
        # pCloudPlatform.init()
        pCloudPlatform.reset()
        time.sleep(30)
        scheduler.shutdown()
    finally:
        pCloudPlatform.closePort()

def usage():
    print('Cloud Platform')

if __name__ == '__main__':

    config =configparser.ConfigParser()
    try:
        config.read_file(open("config.ini", 'r'))
    except:
        print('read config.ini failed!')
        sys.exit(0)

    longitude,latitude = float(config['location']['longitude']),float(config['location']['latitude'])
    print("location: %s, %s"%(longitude,latitude))
    AzimuthSensorIni = float(config['HyperSAS']['AzimuthSensor'])
    print("HypySAS initial azimuth:%.2f"%AzimuthSensorIni)
    AzimuthDelta = float(config['HyperSAS']['AzimuthDelta']) #测量时的sensor 与 太阳的角度差
    print("azimuth delta:%.2f"%(AzimuthDelta))
    StartTime, EndTime = config['HyperSAS']['StartTime'],config['HyperSAS']['EndTime']
    AzimuthSensorMin = float(config['HyperSAS']['AzimuthSensorMin'])
    AzimuthSensorMax = float(config['HyperSAS']['AzimuthSensorMax'])
    print("azimuth range for sensor: %.2f, %.2f" % (AzimuthSensorMin, AzimuthSensorMax))

    Port, Baudrate = config['serial']['Port'], int(config['serial']['Baudrate'])

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--do",help="action of the cloudplatform: left = 1 right = 2 up = 3\n"
                                                       "down =4leftup = 5leftdown = 6rightup = 7\n"
                                                       "rightdown = 8reset = 9",type=int)
    parser.add_argument("-s", "--speed",default = 10,help="speed, (1-63)")
    parser.add_argument("-a", "--auto",default=False,help="automaticlly adjust the angle")

    args = parser.parse_args()

    v_auto = args.auto
    v_action = args.do
    if v_action == None and v_auto == False:
        print('当前非自动模式，请指定动作指令: -d left right ....')
        sys.exit(0)
    pCloudPlatform = CloudPlatform(longitude, latitude,AzimuthSensorIni,AzimuthDelta,AzimuthSensorMin=AzimuthSensorMin,AzimuthSensorMax=AzimuthSensorMax,
                                   port=Port,baudrate=Baudrate)
    pCloudPlatform.setRuntime(StartTime,EndTime)

    if pCloudPlatform.isRuning()==False:
        print('云台没有连接成功，请检查电源！')
        sys.exit(0)

    # print (v_auto)
    if bool(v_auto) == True:
        print('当前将按照自动模式运行！')
        pCloudPlatform.init()
        print('init finished.')
        time.sleep(30)
        pCloudPlatform.first_adjust()
        time.sleep(30)
        autoRun(pCloudPlatform)
        # print(pCloudPlatform.pPort.closed)
        # pCloudPlatform.protocolD.left(10)
        pass

    else:
        # pCloudPlatform.do(action=v_action)
        print('暂未实现,哈哈哈')
        pass






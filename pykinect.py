#!/usr/bin/env python
#-*- encoding: utf-8 -*-
'''
@File    : pykinect.py
@Time    : 2019/04/29 09:53
@Author  : Jingsheng Tang
@Version : 1.0
@Contact : mrtang@nudt.edu.cn   mrtang_cs@163.com
@License : (C) All Rights Reserved

@description: module for getting video stream and point cloud, and for positioning target 
'''

import os,sys
import mmap
import numpy as np
import pygame
from pygame.locals import *
import time
from struct import pack,unpack
import cv2
import win32api
from dragdraw_pgrect import DragDraw_pgRect
import win32com.client

rootdir = os.path.dirname(os.path.abspath(__file__))
server_path = os.path.join(rootdir,r'KinectBase source\KinectDriver\bin\Debug')

## -------------------------------------------------------------------------------------------------------------------
## kinect client

class KinectClientV2019(object):
    """
    author: mrtang
    date: 2017.5
    version: 1.0
    email: mrtang@nudt.edu.cn
    
    update:
        added opencv support
    
    A Kinect server runs in another process to capture RGB video and depth image,
    and futrher recoginze accessible are, self pose and detect interesting buttons,
    this information were ongoing written in shared memeory in special format.
    This class is used to read and unpack these data.

    """
    def __init__(self,kpath = server_path):  #kpath = '',将不在此启动kinectbase,也不会释放kinectbase
        self.servername = 'KinectBase.exe'
        self.release = False
        if kpath != '':
            if not check_exsit(self.servername):    os.startfile(os.path.join(kpath, self.servername))
            print 'kinect driver is running...'
            self.release = True
            time.sleep(2)

        self.SHrgb = mmap.mmap(0,921604,access=mmap.ACCESS_READ,tagname='_sharemem_for_colorpixels_')
        self.SHdepth = mmap.mmap(0,921604,access=mmap.ACCESS_READ,tagname='_sharemem_for_depthpixels_')
        self.SHpointcloud = mmap.mmap(0,3686404,access=mmap.ACCESS_READ,tagname='_sharemem_for_point_cloud_') #double

        self.color_surface = pygame.surface.Surface((640,480))
        self.depth_surface = pygame.surface.Surface((640,480))
        self.pointcloud = np.zeros((480,640,3))
        self.__color_index = self.__depth_index = self.__pc_index = 0

        self.color_buf = '\x00'*921600
        self.depth_buf = '\x00'*921600
        self.color_cvframe = self.depth_cvframe = cv2.imread(os.path.join(rootdir,'kinect.jpg'))

    def __del__(self):
        if self.release:
            if check_exsit(self.servername):    kill_process(self.servername)

    def update_color_buf(self):
        self.SHrgb.seek(0)
        newind = unpack('i',self.SHrgb.read(4))[0]
        if newind!=self.__color_index:
            self.__color_index = newind
            self.color_buf = self.SHrgb.read(921600)

    def get_color_as_pgsurface(self):
        self.update_color_buf()
        self.color_surface = pygame.image.frombuffer(self.color_buf,(640,480),'RGB').convert()
        return self.color_surface

    def get_color_as_cvframe(self):
        self.update_color_buf()
        arr = np.fromstring(self.color_buf,np.uint8)
        self.color_cvframe = arr.reshape((480,640,3))
        self.color_cvframe = cv2.cvtColor(self.color_cvframe, cv2.COLOR_BGR2RGB)
        return self.color_cvframe
        
    def update_depth_buf(self):
        self.SHdepth.seek(0)
        newind = unpack('i',self.SHdepth.read(4))[0]
        if newind!=self.__depth_index:
            self.__depth_index = newind
            self.depth_buf = self.SHdepth.read(921600)
        return self.depth_buf

    def get_depth_as_pgsurface(self):
        self.update_depth_buf()
        self.depth_surface = pygame.image.frombuffer(self.depth_buf,(640,480),'RGB').convert()
        return self.depth_surface

    def get_depth_as_cvframe(self):
        self.update_depth_buf()
        self.depth_cvframe = np.fromstring(self.depth_buf,np.uint8).reshape((480,640,3))
        self.depth_cvframe = cv2.cvtColor(self.depth_cvframe, cv2.COLOR_BGR2RGB)
        return self.depth_cvframe

    @property
    def point_cloud(self):
        self.SHpointcloud.seek(0)
        newind = unpack('i',self.SHpointcloud.read(4))[0]
        if newind!=self.__pc_index:
            self.__pc_index = newind
            self.SHpointcloud.seek(4)
            self.pointcloud = np.fromstring(self.SHpointcloud.read(3686400),dtype=np.float32).reshape((480,640,3))
            self.pointcloud[450:,:,:] = 0
        return self.pointcloud

    def get_xyz(self,p): #p：x,y
        return self.point_cloud[p[1],p[0],:]
        

## -------------------------------------------------------------------------------------------------------------------
## calibration

def calibration():
    '''
    it is used for calibrating the mounting parameters of the camera.
    we will fit the floor and the wall, to calculate their normal vectors.
    these two normal vectors and the third orthogonal vector define the
    target world coordinate system. called transmit matrix.
    ref: http://blog.csdn.net/u011240016/article/details/52821139
    '''

    while True:
        cflg = int(raw_input(u'\n'
                             u'aim to calibrate the mounting parameters of the camera\n'
                             u'clear any parameters of camera\n'
                             u'input：\n'
                             u'1: calibrate the floor\n'
                             u'2: calibrate the wall\n'
                             u'3: calculate and save parameters\n'
                             u'0: quit this program\n'
                             u'\n'))
        if cflg == 0: break

        if cflg in [1,2]:
            pygame.init()
            screen = pygame.display.set_mode((640, 480))
            END = False
            dr = DragDraw_pgRect(screen)
            kk = KinectClientV2019()

            while not END:
                screen.blit(kk.get_color_as_pgsurface(),(0,0))
                events = pygame.event.get()
                flg, rec = dr.update(events)

                for ev in events:
                    if ev.type == pygame.QUIT:
                        END = True
                    elif ev.type == pygame.KEYUP and ev.key == 13:  # enter key
                        if flg:
                            coe = plane_fit(kk.point_cloud,rec)
                            if cflg == 1:
                                np.save('ground_raw.npy', coe)
                                coe = coe[:3]
                                coe *= np.sign(coe[1])  #令法向量的y为正，即指向地面
                                coe *= 1/np.linalg.norm(coe) #归一化
                                np.save('ground.npy',coe)
                                win32api.MessageBox(0, u'地面标定完成，关闭窗口继续')

                            elif cflg == 2:
                                coe = coe[:3]
                                coe *= np.sign(coe[2])  # 令法向量的z为正，即指向前方
                                coe /= np.linalg.norm(coe)  # 归一化
                                np.save('wall.npy', coe)
                                win32api.MessageBox(0, u'墙面标定完成，关闭窗口继续')

                pygame.display.update()
            pygame.quit()
            del kk

        elif cflg == 3:
            #通过两个方向定义第三个方向
            ax, ay, az = coeY = np.load('ground.npy')
            fY = np.load('ground_raw.npy')
            bx, by, bz = coeZ = np.load('wall.npy')
            cx = ay * bz - az * by
            cy = -az * bx + ax * bz
            cz = ax * by - ay * bx
            v = np.array((cx, cy, cz))
            coeX = v/np.linalg.norm(v)

            buf = fY.astype(np.float32).tostring()
            with open(os.path.join(rootdir, 'groundfilter_param.txt'), 'w') as f:
                f.write(buf)

            pbuf = np.hstack((coeX, coeY, coeZ)).astype(np.float32).tostring()
            with open(os.path.join(rootdir, 'installtion_param.txt'), 'w') as f:
                f.write(pbuf)

            win32api.MessageBox(0, u'计算完成，关闭窗口继续')


## -------------------------------------------------------------------------------------------------------------------
## opencv demo 

def demo_cv():
    '''
    a simple demo for presenting how to show color stream with opencv
    '''
    kk = KinectClientV2019()
    while True:
        frame = kk.get_color_as_cvframe()
        frame = kk.get_depth_as_cvframe()
        cv2.imshow("capture", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    del kk
    cv2.destroyAllWindows()

## -------------------------------------------------------------------------------------------------------------------
## pygame demo

def demo_pg():
    '''
    a simple demo for presenting how to show color stream with pygame
    '''
    pygame.init()
    clk = pygame.time.Clock()
    kk = KinectClientV2019()
    screen = pygame.display.set_mode((1280,480), 0,32)
    END=0
    while not END:
        screen.blit(kk.get_color_as_pgsurface(),(640,0))
        screen.blit(kk.get_depth_as_pgsurface(),(0,0))
        ev = pygame.event.get()
        for e in ev:
            if e.type == QUIT:
                END=1
            elif e.type == MOUSEBUTTONUP:
                x,y = e.pos
                x %= 640
                y %= 480
                print x,y
        pygame.display.update()
        clk.tick(60)
    del kk
    pygame.quit()

## -------------------------------------------------------------------------------------------------------------------
## locate point

def demo_getXYZ():
    '''
    a simple demo for presenting how to get the world position in the image
    '''
    pygame.init()
    clk = pygame.time.Clock()
    kk = KinectClientV2019()
    screen = pygame.display.set_mode((1280,480), 0,32)
    END=0
    while not END:
        screen.blit(kk.get_color_as_pgsurface(),(640,0))
        screen.blit(kk.get_depth_as_pgsurface(),(0,0))
        ev = pygame.event.get()
        for e in ev:
            if e.type == QUIT:
                END=1
            elif e.type == MOUSEBUTTONUP:
                x,y = e.pos
                x %= 640
                y %= 480
                print '>>> x: %d  y: %d'%(x,y)
                print '>>> world position: %s'%(str(kk.point_cloud[y,x]))
        pygame.display.update()
        clk.tick(60)
    del kk
    pygame.quit()


## -------------------------------------------------------------------------------------------------------------------
## locate bottle

def locate_bottle(rect, point_cloud):
    '''
    a function for locating small object such as bottle
    '''
    (x,y),(w,h) = rect
    rw = 0.7 if w < 20 else 0.6
    rh = 0.7 if h < 30 else 0.6     #缩小目标box

    x = int(x + w/2 - rw * w/2)
    y = int(y + h/2 - rh * h/2)
    w = int(w * rw)
    h = int(h * rh)    #缩小后的box

    pc = point_cloud[y:y+h, x:x+w, :]
    s = pc.shape
    pp = pc.flatten().reshape((s[0] * s[1], 3))
    pp = pp[np.where(pp[:,2] > 0)]   #去除了无效点
    d = np.linalg.norm(pp,axis=-1)
    ind = np.argsort(d)[0]
    return pp[ind,:]    #定位瓶子时由于小目标容易切割到桌面上的点，因此定位距离最近的点


## -------------------------------------------------------------------------------------------------------------------
## locate obj
def locate_obj(rect, point_cloud):
    '''
    a function for locating objects
    '''
    (x,y),(w,h) = rect
    rw = 0.7 if w < 20 else 0.6
    rh = 0.7 if h < 30 else 0.6     #缩小目标box

    x = int(x + w/2 - rw * w/2)
    y = int(y + h/2 - rh * h/2)
    w = int(w * rw)
    h = int(h * rh)    #缩小后的box

    pc = point_cloud[y:y+h, x:x+w, :]
    s = pc.shape
    pp = pc.flatten().reshape((s[0] * s[1], 3))
    pp = pp[np.where(pp[:,2] > 0)]   #去除了无效点
    return np.mean(pp,axis = 0)
    

## -------------------------------------------------------------------------------------------------------------------
## plane fit functions

def convert(pc):  # pc r*c*3 pointcloud ndarray 整理切割过来的点云
    s = pc.shape
    pp = pc.flatten().reshape((s[0] * s[1], 3)).transpose()
    ind = np.where(pp[2, :] == 0)[0]
    pp = np.delete(pp, ind, axis=1)     #去除无效点
    return np.asmatrix(pp)  # return x*3 matrix


def plane_fitting(pp):  # 3*x  对一组3空间点拟合法向量
    '''
    raw function to fit plane points to get normal vector
    '''
    cpp = np.cov(pp)
    a, v = np.linalg.eig(cpp)
    ind = np.argsort(a)[0]
    vv = v[:, ind]
    mvv = np.asmatrix(vv).T
    m = -(np.mean(pp, axis=1).T * mvv)[0, 0]  # bias
    return np.hstack((vv, m))


def hfiting(pp):    # 带滤噪的平面拟合
    '''
    optimized plane fit method
    '''
    while True:
        coe = plane_fitting(pp)
        num = pp.shape[1]
        v = np.asmatrix(coe[:3])
        ds = v * pp
        md = np.mean(ds)
        nds = ds - md
        threshold = 3 * np.sqrt(nds * nds.T / (num - 1))  # 去除了异常点
        ind = np.where(nds > threshold)[1]
        if ind.size == 0: break
        pp = np.delete(pp, ind, axis=1)
    return coe

def plane_fit(point_cloud,rec):
    [x,y],[w,h] = rec
    pc = point_cloud[y:y+h, x:x+w, :]
    pp = convert(pc)
    return hfiting(pp)


## plane fit demo

def demo_plane_fit():
    '''
    a demo to fit a plane
    draw a rectange and press enter key
    '''
    pygame.init()
    screen = pygame.display.set_mode((640, 480))
    END = False
    dr = DragDraw_pgRect(screen)
    kk = KinectClientV2019()

    while not END:
        screen.blit(kk.get_color_as_pgsurface(),(0,0))
        events = pygame.event.get()
        flg, rec = dr.update(events)

        for ev in events:
            if ev.type == pygame.QUIT:
                END = True
            elif ev.type == pygame.KEYUP and ev.key == 13:  # enter key
                if flg:
                    coe = plane_fit(kk.point_cloud,rec)
                    print '>>> normal vector of this plane'
                    print str(coe)
                    # buf = coe.astype(np.float32).tostring()
                    # with open(os.path.join(rootdir,'groundfilter_param.txt'),'w') as f:
                        # f.write(buf)
                        # print '>>> plane fitting succeed'
        pygame.display.update()
    pygame.quit()


## -------------------------------------------------------------------------------------------------------------------
## 

def check_exsit(process_name):
    '''
    check if a process is exist
    '''
    WMI = win32com.client.GetObject('winmgmts:') 
    processCodeCov = WMI.ExecQuery('select * from Win32_Process where Name="%s"' % process_name) 
    if len(processCodeCov) > 0:return 1
    else:return 0

def kill_process(process_name):
    '''
    kill a process by name
    '''
    if os.system('taskkill /f /im ' + process_name)==0:return 1
    else:return 0


## -------------------------------------------------------------------------------------------------------------------
## target locate demo

def demo_target_locate():
    '''
    a demo for presenting how to locate object
    '''
    pygame.init()
    screen = pygame.display.set_mode((1280,480))
    END = False
    dr = DragDraw_pgRect(screen)
    kk = KinectClientV2019()

    while not END:
        screen.blit(kk.get_color_as_pgsurface(),(0,0))
        screen.blit(kk.get_depth_as_pgsurface(),(640,0))
        events = pygame.event.get()
        flg,rec = dr.update(events)

        for ev in events:
            if ev.type == pygame.QUIT:
                END = True
            elif ev.type == pygame.KEYUP and ev.key == 13:  # enter key
                if flg:
                    print 'you have seleted the area: %s'%(str(rec))
                    c = locate_obj(rec, kk.point_cloud)
                    print str(c)
                else:
                    print "you havn't selected any region"
        pygame.display.update()
    del kk
    pygame.quit()
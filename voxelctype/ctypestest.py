# -*- coding: utf-8 -*-
import platform
from ctypes import *
import cv2
import numpy as np
import time

if platform.system() == 'Windows':
    libvoxel = cdll.LoadLibrary('voxelpydll.dll')

elif platform.system() == 'Linux':
    libvoxel = cdll.LoadLibrary('voxelpydll.so')

print(libvoxel)

ret = libvoxel.tofInit()
libvoxel.tofReadDepth.restype = POINTER(c_float)

time.sleep(5)
if ret == 0:
    while(1):
        #dv = c_float(320*240)
        dv = libvoxel.tofReadDepth()
        #print(dv[0])

        np_data = np.ctypeslib.as_array(dv, (240, 320))
        #np_data = np_data.reshape((240, 320))
        cv2.imshow('a', np_data)
        key = cv2.waitKey(30)
        if key == 27:
            break

quit(0)
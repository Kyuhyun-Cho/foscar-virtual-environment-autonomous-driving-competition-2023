#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Point
from std_msgs.msg import *
import numpy as np
from math import cos,sin,sqrt,pow,atan2,pi

class pathReader :
    def __init__(self,pkg_name):
        rospack=rospkg.RosPack()
        self.file_path=rospack.get_path(pkg_name)

    def read_txt(self,file_name):
        full_file_name=self.file_path+"/path/"+file_name
        openFile = open(full_file_name, 'r')
        
        out_path=[]

        line=openFile.readlines()
        for i in line :
            tmp=i.split()
            
            read_pose = np.zeros((3, 1))
            read_pose = np.array([[float(tmp[0])], [float(tmp[1])], [float(tmp[2])]], dtype=np.float64)
            out_path.append(read_pose)
        
        openFile.close()
        return out_path
    
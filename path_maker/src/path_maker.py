#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import warnings
import os
import sys
import rospy
import rospkg
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix, Imu
from morai_msgs.msg  import EgoVehicleStatus, GPSMessage
from math import pi, sqrt, atan2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
import tf

warnings.simplefilter(action='ignore', category=FutureWarning)

class PathMaker :

    def __init__(self):
        rospy.init_node('path_maker', anonymous=True)

        arg = rospy.myargv(argv = sys.argv)
        self.path_folder_name = arg[1]
        self.make_path_name = arg[2]

        rospy.Subscriber("/imu", Imu, self.imuCB)
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
    
        self.cnt = 0
        
        self.is_imu = False
        self.is_gps = False
        self.prev_x = 0
        self.prev_y = 0

        # ENU 좌표계로 변환하기 위한 값 
        self.x_offset = 322944.1350182715 
        self.y_offset = 4164675.786437934

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        rospack=rospkg.RosPack()
        pkg_path = rospack.get_path('path_maker')
        full_path = pkg_path +'/'+ self.path_folder_name+'/'+self.make_path_name+'.txt'
        self.f = open(full_path, 'w')

        rate = rospy.Rate(30) 
        while not rospy.is_shutdown():
            if self.is_imu == True and self.is_gps == True:
                self.path_make()
            rate.sleep()    

        self.f.close()
        

    def path_make(self):
        x = self.xy_zone[0]- self.x_offset
        y = self.xy_zone[1]- self.y_offset
        yaw = self.yaw

        distance = sqrt(pow(x - self.prev_x, 2) + pow(y - self.prev_y, 2))

        if distance > 0.05:   #0.3
            data='{0}\t{1}\t{2}\n'.format(x, y, yaw)
            self.f.write(data)
            self.prev_x = x
            self.prev_y = y
            self.cnt += 1
        
            print(self.cnt, 'X:', x, 'Y:', y, 'YAW:', yaw)
            
            
    def imuCB(self, data): 
        w = data.orientation.w
        x = data.orientation.x
        y = data.orientation.y
        z = data.orientation.z

        self.yaw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        self.is_imu = True
        

    def gpsCB(self, data): 
        self.xy_zone = self.proj_UTM(data.longitude, data.latitude)
        
        self.is_gps = True

        
if __name__ == '__main__':
    try:
        path_maker_=PathMaker()
    except rospy.ROSInterruptException:
        pass




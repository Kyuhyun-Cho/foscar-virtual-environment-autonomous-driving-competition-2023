#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker, MarkerArray
from morai_msgs.msg import GetTrafficLightStatus, GPSMessage, EgoVehicleStatus
from utils import pathReader,findLocalPath,purePursuit,latticePlanner
from pyproj import Proj
import tf
from math import *

MAX_SPEED = 8

class Point():
    def __init__(self, min_x, min_y, max_x, max_y):
        self.x1 = min_x
        self.y1 = min_y
        self.x2 = max_x
        self.y2 = max_y


class WeCarPlanner():
    def __init__(self):
        rospy.init_node('wecar_planner', anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        
        if len(arg) < 2:
            self.path_name = 'global_path.txt'
        else:
            self.path_name = arg[1]


        # Publisher
        self.global_path_pub =      rospy.Publisher('/global_path', Path, queue_size=1) 
        self.local_path_pub =       rospy.Publisher('/local_path', Path, queue_size=1) 
        self.target_waypoint_pub =  rospy.Publisher('/target_waypoint', Marker, queue_size=1)
        self.motor_pub =            rospy.Publisher('commands/motor/speed',Float64, queue_size=1)
        self.servo_pub =            rospy.Publisher('commands/servo/position',Float64, queue_size=1)
        
        
        # Subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.Subscriber("/imu", Imu, self.imuCB)
        
        # rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, self.trafficLightCB) 
        ############################## 신호등 토픽 분석 ##############################
        # trafficLightIndex -> SN000000, SN000002, SN000005, SN000009 (반시계방향)
        # trafficLightStatus:
        # 빨간불: 1
        # 초록불: 16
        # 노란불: 4
        # 빨간불 + 좌회전: 33
        # 빨간불 + 노란불: 5         
        ###########################################################################
        
        
        ########################  lattice  ########################
        for i in range(1,8):            
            globals()['lattice_path_{}_pub'.format(i)]=rospy.Publisher('lattice_path_{}'.format(i),Path,queue_size=1)  
        ########################  lattice  ########################
 

        # Hyper Parameter
        self.steering_angle_to_servo_offset=0.5 ## servo moter offset
        

        # Variable
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        
        self.target_x = 0
        self.target_y = 0

        self.x_offset = 322944.1350182715 
        self.y_offset = 4164675.786437934

        self.is_gps = False
        self.is_imu = False
        
        self.euler_data = [0,0,0,0]
        
        
        # TF
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        
        # Message
        self.motor_msg = Float64()
        self.servo_msg = Float64()
        self.status_msg = EgoVehicleStatus()
    

        # Class
        path_reader =   pathReader('path_maker') ## 경로 파일의 위치
        pure_pursuit =  purePursuit() ## purePursuit import
    

        # Read path
        self.global_path=path_reader.read_txt(self.path_name) ## 출력할 경로의 이름
    
        rate = rospy.Rate(30) # 30hz
                                           
        #  0 1 2 -> 총 3개의 lattice 중 가운데 lattice == 1
        lattice_current_lane = 1
        while not rospy.is_shutdown():
            
            if self.is_gps and self.is_imu: ## WeBot 상태, 장애물 상태 점검

                ## global_path와 WeBot status_msg를 이용해 현재 waypoint와 local_path를 생성
                local_path,self.current_waypoint=findLocalPath(self.global_path,self.status_msg) 


                ########################  lattice  ########################
                vehicle_status=[self.status_msg.position.x, self.status_msg.position.y, (self.status_msg.heading)/180*pi, MAX_SPEED/3.6]
                # lattice_path, selected_lane = latticePlanner(local_path, vehicle_status, lattice_current_lane)
                # lattice_current_lane = selected_lane

                # # 최소 가중치를 갖는 lattice path 선택하기 
                # if selected_lane != -1: 
                #     local_path = lattice_path[selected_lane]
                
                
                # # lattice path visualization을 위한 path publish 과정
                # if len(lattice_path)==3:                    
                #     for i in range(1,4):
                #         globals()['lattice_path_{}_pub'.format(i)].publish(lattice_path[i-1])
                ########################  lattice  ########################

                pure_pursuit.getPath(local_path) ## pure_pursuit 알고리즘에 Local path 적용
                pure_pursuit.getEgoStatus(self.status_msg) ## pure_pursuit 알고리즘에 WeBot status 적용

                self.steering, self.target_x, self.target_y = pure_pursuit.steering_angle()

                # 조향 값 확인 : rostopic echo /sensors/servo_position_command -> data
                # range : 0.0 ~ 1.0 (straight 0.5)
                
                # self.servo_msg = self.steering*0.021 + self.steering_angle_to_servo_offset  # 조향값
                self.servo_msg = self.steering*0.027 + self.steering_angle_to_servo_offset  # 조향값
    
                servo_degree = abs(self.servo_msg - 0.5)

                self.motor_msg = 8
                
                # if servo_degree > 0.2:
                #     self.motor_msg = 5
                # elif servo_degree > 0.1:
                #     self.motor_msg = 6
                # else: self.motor_msg = 8 



                # Local Path 출력
                # self.local_path_pub.publish(local_path)
                
                # self.visualizeTargetPoint(self.target_x, self.target_y)
                
                self.publishMotorServoMsg(self.motor_msg, self.servo_msg)
            
                self.global_path_pub.publish(self.global_path)
                
                rate.sleep()    
                
            else:
                print("GPS or IMU is not working")
                
        
    def gpsCB(self, msg):
        utmk_xy = self.proj_UTM(msg.longitude, msg.latitude)
        
        self.status_msg.position.x = utmk_xy[0] - self.x_offset # ENU x
        self.status_msg.position.y = utmk_xy[1] - self.y_offset # ENU y
        
        self.tf_broadcaster.sendTransform((self.status_msg.position.x, self.status_msg.position.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, (self.status_msg.heading)/180*pi),
                        rospy.Time.now(),
                        "base_link",
                        "map")
        
        
        self.is_gps = True
        
        
    def imuCB(self, msg):
        w = msg.orientation.w
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z

        yaw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2))) # 라디안 단위
        
        self.status_msg.heading = yaw*180/pi
        self.status_msg.velocity.x = MAX_SPEED
        
        self.is_imu = True
        

    def isMissionArea(self, x1, y1, x2, y2):
        if (x1 <= self.status_msg.position.x <= x2) and (y1 <= self.status_msg.position.y <= y2):
            return True
        else:
            return False
     

    # def trafficLightCB(self, msg) :    
    #     if self.isMissionArea(self.traffic_area.x1, self.traffic_area.y1, self.traffic_area.x2, self.traffic_area.y2):
    #         if msg.trafficLightStatus == 16 : # green light == 16
    #             self.traffic_greenlight = True
    #         else : 
    #             self.traffic_greenlight = False

    

    def publishMotorServoMsg(self, motor_msg, servo_msg):
        motor_msg = motor_msg * 300
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_msg)


    def visualizeTargetPoint(self, x, y):
        target_waypoint = Marker()
        target_waypoint.header.frame_id = "map"
        target_waypoint.id = 1001
        target_waypoint.type = target_waypoint.SPHERE
        target_waypoint.action = target_waypoint.ADD
        target_waypoint.scale.x = 0.1
        target_waypoint.scale.y = 0.1
        target_waypoint.scale.z = 0.1
        target_waypoint.color.a = 1.0
        target_waypoint.color.r = 0.6
        target_waypoint.color.g = 0.7
        target_waypoint.color.b = 0.8
        target_waypoint.pose.orientation.w = 1.0
        target_waypoint.pose.position.x = x
        target_waypoint.pose.position.y = y
        target_waypoint.pose.position.z = 0.0
        target_waypoint.lifetime = rospy.Duration(0.1)
        self.target_waypoint_pub.publish(target_waypoint)



if __name__ == '__main__':
    try:
        wecar_planner=WeCarPlanner()
    except rospy.ROSInterruptException:
        pass
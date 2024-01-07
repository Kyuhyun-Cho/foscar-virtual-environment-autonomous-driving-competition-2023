#! /usr/bin/env python3

import sys, os
import yaml
import numpy as np
from math import atan2, sin, cos, pi

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
from morai_msgs.msg import GPSMessage

from pyproj import Proj

from mpc_path_tracking import mpc_path_tracking
from utils import pathReader


SCAILING = 10 # throttle scailing을 위한 상수
MAX_SPEED = 8

class mpc_core:
    def __init__(self):
        # ros parameter
        receding = 10 # 현재 시점부터 어느 범위까지 미래를 예측할지
        max_speed = MAX_SPEED * 300 # 차량의 최대 속도 (하드웨어적 사양)

        ref_speed_scale_down = 7
        ref_speed = ref_speed_scale_down * 300 # 차량이 지향하고자 하는 참조 속도
        
        max_acce = 100  # 차량의 최대 가속도 제한, 속도를 너무 빠르게 증가시키 못하도록, 안정성을 위해서
        max_acce_steer = 0.2 # 차량의 최대 조향 가속도 제한, 방향을 너무 빠르게 바꾸지 못하도록, 안정성을 위해서
        sample_time = 0.1 # 차량의 제어 입력을 어마나 자주 업데이트 할지, 값이 크면 반응이 느려지고 너무 작으면 계산 부하가 증가함
        max_steer = 1.0 # 차량의 최대 조향 각도 제한, 너무 급격하게 방향을 바꾸는 것을 방지하기 위해서
        self.wheelbase = 0.3 # 축거
        
        self.steer_offset = 0.5

        # 가중치 정의
        # control state: 현재 상태와 미래의 예측 상태간의 차이에 적용, 값이 클수록 상태의 변화를 작게 유지하려고 함
        cs = [1, 1, 1] 
        cs = np.diag(cs)

        # control input: 제어 입력의 크기에 적용, 값이 클수록 제어 입력의 크기를 작게 유지하려고 함 
        cu = 1
        
        # control terminal state: 미래의 특정 시점에서의 상태와 목표 상태간의 차이에 적용, 값이 클수록 최종 상태를 목표 상태에 가깝게 유지하려고 함
        cst =[1, 1, 1] 
        cst = np.diag(cst)
        
        # control terminal input: 미래의 특정 시점에서의 제어 입력에 적용, 값이 클수록 최종 제어 입력을 작게 유지하려고 함 
        cut = 1

        # mpc
        self.mpc_track = mpc_path_tracking(receding=receding, max_speed=max_speed, ref_speed=ref_speed, max_acce=max_acce, max_acce_steer=max_acce_steer, sample_time=sample_time, max_steer=max_steer, wheelbase=self.wheelbase, cs=np.diag([1, 1, 1]), cu=cu, cst=cst, cut=cut)

        self.throotle = Float64()
        self.steer = Float64() # 차량 제어 메시지 타입 -> WeBot 제어 메시지 타입으로 변경 예정
        
        self.robot_state = [[0.0], 
                            [0.0], 
                            [0.0]]  # 차량 초기 state -> [0, 0, 0]으로 변경해도 무방할 듯
        
        self.x = self.y = self.z = self.angle = 0
        
        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        # Init node
        rospy.init_node('mpc_node')

        
        # Subscriber
        # rospy.Subscriber('/odom', Odometry, self.robot_state_callback)
        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        
        # Publisher
        # self.pub_throttle = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
        self.pub_throttle = rospy.Publisher('/commands/vel', Float64, queue_size=10)
        self.pub_steer =    rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
        self.pub_path =     rospy.Publisher('/global_path', Path, queue_size=10)
        
        
        # self.ref_path_list = '../path_maker/path/global_path.txt' # ref_path_list == global_path, 우리는 txt 파일로 대체 필요함. [x, y, theta(라디안)] 형식 
        arg = rospy.myargv(argv=sys.argv)

        if len(arg) < 2:
            self.path_name = 'global_path.txt'
        else:
            self.path_name = arg[1]
        
        path_reader=pathReader('path_maker') ## 경로 파일의 위치
        
        self.ref_path_list=path_reader.read_txt(self.path_name) ## 출력할 경로의 이름
        self.path = self.generate_path(self.ref_path_list) # Rviz Visualization용 Path()


    def cal_vel(self):

        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            # print("0 parameter called robotstate", type(self.robot_state))
            # print(self.robot_state)
            opt_vel, _, flag, _ = self.mpc_track.controller(self.robot_state, self.ref_path_list, iter_num=5)
            print('******')
            print('states:\n', self.robot_state)
            
            if flag == True:
                self.throttle = 0
                self.steer = 0 + self.steer_offset
            else:
                self.throttle = opt_vel[0, 0] # / SCAILING
                self.steer =    opt_vel[1, 0] + self.steer_offset

            print('actions:')
            # print('throttle',self.output.throttle , 'steer',self.output.steer)
            print('throttle',self.throttle , 'steer',self.steer)
            self.pub_throttle.publish(self.throttle)
            self.pub_steer.publish(self.steer)
            self.pub_path.publish(self.path)

            rate.sleep()

    def gps_callback(self, data):

        utmk_xy = self.proj_UTM(data.longitude, data.latitude)

        enu_x = utmk_xy[0] - 322944.1350182715
        print(enu_x)
        enu_y = utmk_xy[1] - 4164675.786437934
        print(enu_y)
        enu_z = 0

        self.robot_state[0] = enu_x
        self.robot_state[1] = enu_y


    def imu_callback(self, data):
    
        quat = data.orientation

        raw = mpc_core.quat_to_yaw(quat) # raw는 라디안 단위

        raw_degree = mpc_core.quat_to_yaw(quat)*180/pi # 라디안 -> 각도로 변환

        self.robot_state[2] = raw # 라디안 단위


    # def robot_state_callback(self, data):
         
    #     x = data.pose.pose.position.x
    #     y = data.pose.pose.position.y
    #     z = data.pose.pose.position.z

    #     quat = data.pose.pose.orientation

    #     raw = mpc_core.quat_to_yaw(quat) # raw는 라디안 단위

    #     raw_degree = mpc_core.quat_to_yaw(quat)*180/pi # 라디안 -> 각도로 변환

    #     self.x = x
    #     self.y = y
    #     self.z = z
    #     self.angle = raw_degree

    #     # offset = self.wheelbase / 2 
        
    #     self.robot_state[0] = x
    #     self.robot_state[1] = y
    #     self.robot_state[2] = raw # 라디안 단위
    
    # txt 파일을 읽어서 Path를 generate하는 코드로 수정해야함
    @staticmethod 
    def generate_path(ref_path_list):
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = 'map'

        for i in range(len(ref_path_list)):
            ps = PoseStamped()
            ps.pose.position.x = ref_path_list[i][0, 0]
            ps.pose.position.y = ref_path_list[i][1, 0]
            ps.pose.orientation.w = 1

            path.poses.append(ps)

        return path

    @staticmethod
    def quat_to_yaw(quater):
         
        w = quater.w
        x = quater.x
        y = quater.y
        z = quater.z

        raw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw
    

if __name__ == '__main__':
    mp = mpc_core()
    mp.cal_vel()

#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, CompressedImage, Image
import sensor_msgs.point_cloud2 as pc2
from lidar_cam_fusion.msg import PointInfo

class CreateMatrix:
    def __init__(self, params_cam, params_lidar):
        global RT, proj_mtx
        
        self.params_cam = params_cam
        self.params_lidar = params_lidar
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        
        RT = self.transfromMTX_lidar2cam(self.params_lidar, self.params_cam)
        
        proj_mtx = self.project2img_mtx(self.params_cam)
    
    
    def translationMtx(self, x, y, z):
        
        M = np.array([[1,       0,      0,      x],
                      [0,       1,      0,      y],
                      [0,       0,      1,      z],
                      [0,       0,      0,      1]])
        
        return M


    def rotationMtx(self, yaw, pitch, roll):
        
        R_x = np.array([[1,     0,                  0,                  0],
                        [0,     math.cos(roll),     -math.sin(roll),    0],
                        [0,     math.sin(roll),     math.cos(roll),     0],
                        [0,     0,                  0,                  1]])
        
        R_y = np.array([[math.cos(pitch),       0,      math.sin(pitch),     0],
                        [0,                     1,      0,                   0],
                        [-math.sin((pitch)),    0,      math.cos(pitch),     0],
                        [0,                     0,      0,                   1]])
        
        R_z = np.array([[math.cos(yaw),      -math.sin(yaw),     0,     0],
                        [math.sin(yaw),      math.cos(yaw),      0,     0],
                        [0,                  0,                  1,     0],
                        [0,                  0,                  0,     1]])
        
        
        R = np.matmul(R_x, np.matmul(R_y, R_z)) # x, y, z 계산 순서가 중요함
        
        return R


    def transfromMTX_lidar2cam(self, params_lidar, params_cam):
        
        #Relative position of lidar w.r.t cam
        lidar_pos = [params_lidar.get(i) for i in (["X", "Y", "Z"])]
        cam_pos = [params_cam.get(i) for i in (["X", "Y", "Z"])]
        
        x_rel = cam_pos[0] - lidar_pos[0]
        y_rel = cam_pos[1] - lidar_pos[1]
        z_rel = cam_pos[2] - lidar_pos[2]
        
        R_T = np.matmul(self.translationMtx(x_rel, y_rel, z_rel), self.rotationMtx(np.deg2rad(-90.), 0., 0.))
        R_T = np.matmul(R_T, self.rotationMtx(0., 0., np.deg2rad(-90.)))
        
        #rotate and translate the coordinate of a lidar (역행렬)
        R_T = np.linalg.inv(R_T)
        
        return R_T


    def project2img_mtx(self, params_cam):
        
        #focal lengths
        fc_x = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))
        fc_y = params_cam["WIDTH"]/(2*np.tan(np.deg2rad(params_cam["FOV"]/2)))

        # the center of image
        cx = params_cam["WIDTH"]/2
        cy = params_cam["HEIGHT"]/2
        # cy = 470/2
        
        # transformation matrix from 3D to 2D
        R_f = np.array([[fc_x,  0,      cx],
                        [0,     fc_y,   cy]])
        
        return R_f
    

class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):
        # global cap
        
        self.params_cam = params_cam
        self.params_lidar = params_lidar
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        
        ######################## USB Cam 사용시 ########################
        # cap = cv2.VideoCapture(0, cv2.CAP_V4L)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        ######################## USB Cam 사용시 ########################
        
        self.bbox_cnts = 0
        self.x_mini = []
        self.y_mini = []
        self.z_mini = []
        self.x_maxi = []
        self.y_maxi = []
        self.z_maxi = []
        
        self.bridge = CvBridge()
        
        self.image_pub = rospy.Publisher("/calib_img", Image, queue_size=1)
        
        # self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        
        self.image_sub = rospy.Subscriber("/debug_image", Image, self.img_callback)
        self.pc_sub = rospy.Subscriber("/roi_raw", PointCloud2, self.callback)
        self.bbox_point_sub = rospy.Subscriber("/bbox_point_info", PointInfo, self.bbox_point_callback)
        
        self.rate = rospy.Rate(30)
    
    
    
    
    def bbox_point_callback(self, msg):
 
        self.bbox_cnts = msg.bboxCounts
        
        self.x_mini = list(msg.xMini[:self.bbox_cnts])
        self.y_mini = list(msg.yMini[:self.bbox_cnts])
        self.z_mini = list(msg.zMini[:self.bbox_cnts])
        
        self.x_maxi = list(msg.xMaxi[:self.bbox_cnts])
        self.y_maxi = list(msg.yMaxi[:self.bbox_cnts])
        self.z_maxi = list(msg.zMaxi[:self.bbox_cnts])


    def img_callback(self, msg):
        self.original_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        
    def callback(self, msg):
        
        self.rate.sleep()
        
        # Pointclouds가 있을 때
        if msg.data:
            
            self.xyz_p = self.pointcloud_to_xyz(msg)
            
            # LiDAR 기준 점을 Camera 기준점으로 변환
            self.xyz_c = self.transform_lidar2cam(self.xyz_p)
            
            # Camera 기준 점을 Camera 이미지 위의 점으로 투영
            self.xyi = self.project_pts2img(self.xyz_c)
            
            self.publish_img_with_pc(self.xyi)
        
        # Pointclouds 아무 것도 없을 때는 그냥 카메라 이미지만 Publish
        else:
            imgmsg_without_pc = self.bridge.cv2_to_imgmsg(self.original_img,encoding="bgr8")
    
            self.image_pub.publish(imgmsg_without_pc) 
         

        
        
         
    def pointcloud_to_xyz(self, pointclouds):
        
        xyz_p = np.array(list(pc2.read_points(pointclouds, skip_nans=True)))
  
        xyz_p = xyz_p[:, :3]

        return xyz_p
        
    
    def merge_bbox_points(self, x_mini, y_mini, z_mini, x_maxi, y_maxi, z_maxi):
        
        bbox_xyz_p = []
 
        for i in range(self.bbox_cnts):
            bbox_xyz_p.append([x_mini[i], y_mini[i], z_mini[i]])
            bbox_xyz_p.append([x_maxi[i], y_maxi[i], z_maxi[i]])
        
        bbox_xyz_p = np.array(bbox_xyz_p)
        
        return bbox_xyz_p  
        
        
        
        
        
    def transform_lidar2cam(self, xyz_p):
        
        xyz_c = np.matmul(np.concatenate([xyz_p, np.ones((xyz_p.shape[0], 1))], axis=1), RT.T)
        
        return xyz_c
    
    
    def transform_bbox2cam(self, bbox_xyz_p):
        
        bbox_xyz_c = np.matmul(np.concatenate([bbox_xyz_p, np.ones((bbox_xyz_p.shape[0], 1))], axis=1), RT.T)

        return bbox_xyz_c
    
    
    
    
    def project_pts2img(self, xyz_c, crop=True):
        
        xyz_c = xyz_c.T
        
        xc, yc, zc = xyz_c[0,:].reshape([1,-1]), xyz_c[1,:].reshape([1,-1]), xyz_c[2,:].reshape([1,-1])
        
        xn, yn = xc/(zc+0.0001), yc/(zc+0.0001)
        
        xyi = np.matmul(proj_mtx, np.concatenate([xn, yn, np.ones_like(xn)], axis=0))
        
        xyi = xyi[0:2,:].T
        
        if crop:
            xyi = self.crop_pts(xyi)
        else:
            pass
            
        return xyi


    def project_bbox2img(self, bbox_xyz_c, crop=True):
        
        bbox_xyz_c = bbox_xyz_c.T
        
        bbox_xc, bbox_yc, bbox_zc = bbox_xyz_c[0,:].reshape([1,-1]), bbox_xyz_c[1,:].reshape([1,-1]), bbox_xyz_c[2,:].reshape([1,-1])
        
        bbox_xn, bbox_yn = bbox_xc/(bbox_zc+0.0001), bbox_yc/(bbox_zc+0.0001)
        
        bbox_xyi = np.matmul(proj_mtx, np.concatenate([bbox_xn, bbox_yn, np.ones_like(bbox_xn)], axis=0))
        
        bbox_xyi = bbox_xyi[0:2,:].T
    
        # if crop:
        #     bbox_xyi = self.crop_bbox_pts(bbox_xyi)
        # else:
        #     pass
        
        return bbox_xyi




    def crop_pts(self, xyi):
        
        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]
        
        return xyi
    
    
    # def crop_bbox_pts(self, bbox_xyi):
        
    #     bbox_xyi = bbox_xyi[np.logical_and(bbox_xyi[:, 0]>=0, bbox_xyi[:, 0]<self.width), :]
    #     bbox_xyi = bbox_xyi[np.logical_and(bbox_xyi[:, 1]>=0, bbox_xyi[:, 1]<self.height), :]
        
    #     return bbox_xyi
    
    
    
    
    def draw_pts_img(self, img, xi, yi):
    
        point_np = img

        for ctr in zip(xi, yi):
            center = (int(ctr[0]), int(ctr[1]))
            point_np = cv2.circle(point_np, center, 1, (0,255, 0), -1)
            
        return point_np


    def draw_bbox_img(self, img):
        
        self.bbox_xyz_p = self.merge_bbox_points(self.x_mini, self.y_mini, self.z_mini, self.x_maxi, self.y_maxi, self.z_maxi)

        self.bbox_xyz_c = self.transform_bbox2cam(self.bbox_xyz_p)

        self.bbox_xyi = self.project_bbox2img(self.bbox_xyz_c)
        
        bbox_np = img

        for i in range(0, self.bbox_cnts * 2, 2):
           
            bbox_np = cv2.rectangle(bbox_np, (int(self.bbox_xyi[i][0]), int(self.bbox_xyi[i][1])), (int(self.bbox_xyi[i+1][0]), int(self.bbox_xyi[i+1][1])), (255, 0, 0), 1)
    
        return bbox_np
 
        
        
        
    def publish_img_with_pc(self, xyi):
   
        xi, yi = xyi[:, 0].reshape([1,-1]), xyi[:,1].reshape([1,-1])
        xi = xi.flatten()
        yi = yi.flatten()
        
        img_with_pc = self.draw_pts_img(self.original_img, xi, yi)
        
        img_with_pc_bbox = self.draw_bbox_img(img_with_pc)
        
        # result = self.bridge.cv2_to_imgmsg(img_with_pc, encoding="bgr8")
        result = self.bridge.cv2_to_imgmsg(img_with_pc_bbox, encoding="bgr8")
    
        self.image_pub.publish(result) 
        print('Publishing')
        
        
        ######################## USB Cam 사용시 ########################
        # ret, cv_image = self.cap.read()
        
        # if ret:
        #     result = self.draw_pts_img(cv_image, xi, yi)
        #     bridge = cv2.CvBrdige()
        #     self.image_pub.publish(bridge.cv2_to_imgmsg(result, "bgr8"))
        # else:
        #     print("There is no data")
        ######################## USB Cam 사용시 ########################
            


if __name__ == '__main__':
    
    rospy.init_node('lidar_cam_calib', anonymous=True)
    
    ###################################### 센서 파라미터 세팅 ######################################
    params_cam = {"WIDTH": 640, "HEIGHT": 480, "FOV": 90, "X": 4.00, "Y": 0.00, "Z": 0.60}
    params_lidar = {"X": 4.00, "Y": 0.00, "Z": 0.50}
    ###################################### 센서 파라미터 세팅 ######################################
    
    prepare_matrix = CreateMatrix(params_cam, params_lidar)
    
    lidar2cam_transformer = LIDAR2CAMTransform(params_cam, params_lidar)
    
    rospy.spin()
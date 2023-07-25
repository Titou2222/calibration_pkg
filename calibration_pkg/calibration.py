import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image,PointCloud2
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge

import numpy as np
import open3d as o3d
import cv2

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import calib_lib as l

class Calibration(Node):
    def __init__(self):
        super().__init__('calibration_node')   

        # GET EXTERNAL PARAMETERS : ROTATION AND TRANSLATION VECTORS (rvec & tvec)
        self.intrinsic_param = 443.2128, 258.8821, 364.5105, 370.2965 # cx, cy, fx, fy
        self.dist_coeffs = np.array([0,0,0,0,0], dtype=np.float32) # the image is already rectified
        
        objectPoints = np.array([[1.21,1.43,0.18],[1.58,0.49,-0.22],[1.72,0.37,-0.15],[2.0,0.0,0.24],[1.34,-0.71,-0.21],[1.29,-0.87,-0.17]],dtype="float32")
        imagePoints = np.array([[50,167],[364,290],[400,264],[466,192],[681,304],[726,289]],dtype="float32")

        self.rvec, self.tvec = self.get_extrinsic_param(objectPoints, imagePoints, l.camera_intrinsics3x3(self.intrinsic_param))

        # STEREO
        self.image = np.array([])
        self.bridge = CvBridge()
        self.stereo = Image()
        self.stereo_subscriber = self.create_subscription(Image, '/zed2i/zed_node/right/image_rect_color', self.stereo_publi_callback, 10)
        self.image_initialised = False

        #LiDAR
        self.points_3D = np.array([])
        self.pointcloud = o3d.geometry.PointCloud()  
        self.lidar_subscriber = self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_publi_callback, 10)
        self.pointcloud_initialised = False

        #DISPLAY
        self.d_max = 1.6
        self.timer = self.create_timer(0.05, self.display)



    #----------------------------------------------------------------------------------------------------------------#
    #                                         STEREO                                                                 #
    #----------------------------------------------------------------------------------------------------------------#
    def stereo_publi_callback(self,msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_initialised = True

    #----------------------------------------------------------------------------------------------------------------#
    #                                          LiDAR                                                                 #
    #----------------------------------------------------------------------------------------------------------------#
    def lidar_publi_callback(self, msg):
        self.points_3D = point_cloud2.read_points_numpy(msg, skip_nans=True)[:, :3].astype(np.float32)
        mask = np.logical_and(np.logical_and(0.1<self.points_3D[:, 0], self.points_3D[:, 0] <self.d_max), -0.8<self.points_3D[:, 2])
        self.points_3D = self.points_3D[mask]
        self.pointcloud_initialised = True

    #----------------------------------------------------------------------------------------------------------------#
    #                                       CALIBRATION                                                              #
    #----------------------------------------------------------------------------------------------------------------#
    def get_extrinsic_param(self, objectPoints, imagePoints, mat_intrinsic):
        success, rvec, tvec, inliers, = cv2.solvePnPRansac(objectPoints,imagePoints,mat_intrinsic,self.dist_coeffs)

        rot_mat, _ = cv2.Rodrigues(rvec)
        self.get_logger().info('rot_mat:' + str(rot_mat))
        self.get_logger().info('rvec:' + str(rvec))
        self.get_logger().info('tvec:' + str(tvec))
        self.get_logger().info('success:' + str(success))

        return rvec.astype(np.float32), tvec.astype(np.float32)

    def display(self):
        try:
            assert self.pointcloud_initialised==False, "Error : self.points_3D is empty for now."
            assert self.image_initialised==False, "Error : self.image is empty for now."
            return
        except AssertionError as e:
            pass

        if len(self.points_3D) > 4:
            self.points_2D, _ = cv2.projectPoints(self.points_3D,self.rvec, self.tvec, l.camera_intrinsics3x3(self.intrinsic_param), self.dist_coeffs)
            for k in range(len(self.points_2D)):
                x,y = self.points_2D[k][0][0].astype(int),self.points_2D[k][0][1].astype(int)
                if 0<x<len(self.image[0]) and 0<y<len(self.image):
                    cv2.circle(self.image, (x,y), 1, (0, int(l.dist(self.points_3D[k])*255/self.d_max)), 0, -1)
        cv2.imshow("Calibration", self.image)
        cv2.waitKey(1)
            



def main(args=None):
    rclpy.init(args=args)
    calibration = Calibration()
    rclpy.spin(calibration)
    calibration.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


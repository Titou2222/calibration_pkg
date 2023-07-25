import rclpy
from rclpy.node import Node
import numpy as np
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import calib_lib as l
import cv2

class GetExtParam(Node):
    def __init__(self):
        super().__init__('get_ext_param_node')
        # PARAMETERS
        self.intrinsic_param = 443.2128, 258.8821, 364.5105, 370.2965 # cx, cy, fx, fy
        self.dist_coeffs = np.array([0,0,0,0,0], dtype=np.float32)
        
        objectPoints = np.array([[1.53,0.21,0.4],[1.25,-0.23,0.33],[1.75,-0.14,-0.81],[1.48,-0.37,-0.7],        [1.21,1.43,0.18],[1.58,0.49,-0.22],[1.72,0.37,-0.15],     [2.0,0.0,0.24],[1.34,-0.71,-0.21],[1.29,-0.87,-0.17]],dtype="float32")
        imagePoints = np.array([[385,137],[525,100],[553,438],[424,411],    [50,167],[364,290],[400,264],    [466,192],[681,304],[726,289]],dtype="float32")

        self.get_extrinsic_param(objectPoints, imagePoints, l.camera_intrinsics3x3(self.intrinsic_param))

    def get_extrinsic_param(self, objectPoints, imagePoints, mat_intrinsic):
        success, rvec, tvec, inliers, = cv2.solvePnPRansac(objectPoints,imagePoints,mat_intrinsic,self.dist_coeffs)

        rot_mat, _ = cv2.Rodrigues(rvec)
        self.get_logger().info('rot_mat:' + str(rot_mat))
        self.get_logger().info('rvec:' + str(rvec))
        self.get_logger().info('tvec:' + str(tvec))
        self.get_logger().info('success:' + str(success))

def main(args=None):
    rclpy.init(args=args)
    get_ext_param = GetExtParam()
    rclpy.spin(get_ext_param)
    get_ext_param.destroy_node()
    rclpy.shutdown()
    get_ext_param.listener.stop()


if __name__ == '__main__':
    main()


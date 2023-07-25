import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from pynput import keyboard

from typing import List

import cv2
import numpy as np

class OpenModifyImage(Node):
    def __init__(self):
        super().__init__('open_modify_image_node')         
        self.visualize = False # boolean : do the user wants to view the image ?
        self.modify = False
        self.first_time = True
        self.timer = self.create_timer(0.1, self.main)
        self.keypoints = []
        self.print_keypoints = False

        # Image
        self.path = "/home/premove/ros2_ws/save_image/image_5.jpg"
        self.image = cv2.imread(self.path)
        assert self.image is not None, f"Impossible to load image with the path : {self.path}"
        self.sift = cv2.SIFT_create()

        # PARAMETERS
        self.height, self.width = self.image.shape[0],self.image.shape[1]
        self.declare_parameter("xmin", 0)
        self.declare_parameter("xmax", self.width)
        self.declare_parameter("ymin", 0)
        self.declare_parameter("ymax", self.height)

        self.xmin = self.get_parameter("xmin").value
        self.xmax = self.get_parameter("xmax").value
        self.ymin = self.get_parameter("ymin").value
        self.ymax = self.get_parameter("ymax").value
        self.add_on_set_parameters_callback(self.param_callback)

        

        # KEYBOARD
        self.listener = keyboard.Listener(on_press=self.on_key)
        self.listener.start()
        self.get_logger().info('---------------------------------------------')
        self.get_logger().info('/!\ You did not pressed a known key')
        self.get_logger().info('Press the "m" key to modify the image.')
        self.get_logger().info('Press the "v" key to visualize the image.')
        self.get_logger().info('Press the Escape arrow key to close your visualisation.')

    def on_key(self,key):
        try:
            if key.char == "m":
                self.modify = True
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "m" key ...')
                self.get_logger().info("modifying the image")

            elif key.char == "v":
                self.visualize = True
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "v" key ...')
                self.get_logger().info("Opening the visualization")

            elif key.char == "p":
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "p" key ...')
                self.get_logger().info("Printing the keypoints")
                for k in range(len(self.keypoints)):
                    self.get_logger().info(str(self.keypoints[k].pt))
            
            #Xmin
            elif key.char == "k": #Xmin goes <--
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "a" key ...')
                if self.xmin - 5 > 0:
                    self.get_logger().info("Left of scanning zone goes left")
                    self.xmin -= 5
                else:
                    self.get_logger().info("Left of scanning zone cannot goes left more")
                self.get_logger().info("Xmin = " + str(self.xmin))

            elif key.char == "a": #Xmin goes -->
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "k" key ...')
                
                if self.xmin + 5 < self.xmax:
                    self.get_logger().info("Left of scanning zone goes right")
                    self.xmin += 5
                else:
                    self.get_logger().info("Left of scanning zone be at the right of right side")
                self.get_logger().info("Xmin = " + str(self.xmin))

            #Xmax
            elif key.char == "d": #Xmax goes <--
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "d" key ...')
                if self.xmax - 5 > self.xmin:
                    self.get_logger().info("Right of scanning zone goes left")
                    self.xmax -= 5
                else:
                    self.get_logger().info("Right of scanning zone cannot be at the left of left side")
                self.get_logger().info("Xmax = " + str(self.xmax))

            elif key.char == "ñ": #Xmax goes -->
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "ñ" key ...')
                if self.xmax +5 < self.width:
                    self.get_logger().info("Right of scanning zone goes right")
                    self.xmax += 5
                else:
                    self.get_logger().info("Right of scanning zone cannot goes right more")
                self.get_logger().info("Xmax = " + str(self.xmax))
            #Ymin
            elif key.char == "o":
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "w" key ...')
                if self.ymin -5 > 0:
                    self.get_logger().info("Top of scanning zone goes up")
                    self.ymin -= 5
                else:
                    self.get_logger().info("Top of scanning zone cannot goes up more")
                self.get_logger().info("Ymin = " + str(self.ymin))

            elif key.char == "w":
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "o" key ...')
                
                if self.ymin + 5 < self.ymax:
                    self.get_logger().info("Top of scanning zone goes down")
                    self.ymin += 5
                else:
                    self.get_logger().info("Top of scanning zone be lower than bottom of it")
                self.get_logger().info("Ymin = " + str(self.ymin))

            #Ymax
            elif key.char == "s":
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "s" key ...')
                if self.ymax -5 > self.ymin:
                    self.get_logger().info("Bottom of scanning zone goes up")
                    self.ymax -= 5
                else:
                    self.get_logger().info("Bottom of scanning zone cannot be higher than Top of it")
                self.get_logger().info("Ymax = " + str(self.ymax))

            elif key.char == "l":
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "l" key ...')
                if self.ymax +5 < self.height:
                    self.get_logger().info("Bottom of scanning zone goes down")
                    self.ymax += 5
                else:
                    self.get_logger().info("Bottom of scanning zone cannot goes down more")
                self.get_logger().info("Ymax = " + str(self.ymax))

            
            
            elif key == keyboard.Key.esc:
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the escape key ...')
                self.get_logger().info("Closing the visualization")

            else:
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('/!\ You did not pressed a known key')
                self.get_logger().info('Press the "m" key to modify the image.')
                self.get_logger().info('Press the "v" key to visualize the image.')
                self.get_logger().info('Press the Escape arrow key to close your visualisation.')
        except:
            self.get_logger().info("Press 'm', 'p' or 'v'")

    def param_callback(self, params: List[Parameter]):
        for param in params:
            if param.name == "xmin":
                self.xmin = param.value
            if param.name == "xmax":
                self.xmax = param.value

            if param.name == "ymin":
                self.ymin = param.value
            if param.name == "ymax":
                self.ymax = param.value

            if param.name == "zmin":
                self.zmin = param.value
            if param.name == "zmax":
                self.zmax = param.value
        
        return SetParametersResult(successful=True)

    def main(self):
        if self.modify == True:
            

            img_gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
            keypoints_image = np.zeros_like(img_gray)
            img_blur = cv2.GaussianBlur(img_gray, (3,3), 0) 
            image_rognee = img_blur[self.ymin:self.ymax, self.xmin:self.xmax]
            keypoints_image[self.ymin:self.ymax, self.xmin:self.xmax] = image_rognee

            self.keypoints = self.sift.detect(keypoints_image, None)
        if self.visualize == True:
            cv2.imshow('Image with interest points', cv2.drawKeypoints(self.image, self.keypoints, None, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS))
            cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)
    open_modify_image = OpenModifyImage()
    rclpy.spin(open_modify_image)
    open_modify_image.vis.destroy_window()
    open_modify_image.destroy_node()
    rclpy.shutdown()
    open_modify_image.listener.stop()


if __name__ == '__main__':
    main()

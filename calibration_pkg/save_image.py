import rclpy
from rclpy.node import Node

from pynput import keyboard

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SaveImage(Node):
    def __init__(self):
        super().__init__('save_image_node') 
 
        self.image_subscriber = self.create_subscription(Image, '/zed2i/zed_node/right/image_rect_color', self.callback, 10)
        
        self.save = False # boolean : do the user wants to save the image ?
        self.visualize = False # boolean : do the user wants to view the image ?
        self.num = 0 #number of image saves

        self.bridge = CvBridge()

        self.path = "/home/premove/ros2_ws/save_image/image_" + str(self.num) + ".jpg"
       

        # KEYBOARD
        self.listener = keyboard.Listener(on_press=self.on_key)
        self.listener.start()
        self.get_logger().info('---------------------------------------------')
        self.get_logger().info('/!\ You did not pressed a known key')
        self.get_logger().info('Press the "s" key to save the image.')
        self.get_logger().info('Press the "v" key to visualize the image.')
        self.get_logger().info('Press the Escape arrow key to close your visualisation.')

    def on_key(self,key):
        try:
            if key.char == "s":
                self.save = True
                self.num += 1
                self.path = "/home/premove/ros2_ws/save_image/image_" + str(self.num) + ".jpg"
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "s" key ...')
                self.get_logger().info("Image saved at " + self.path + "/home/premove/ros2_ws/save_image/image_" + str(self.num) + ".jpg")

            elif key.char == "v":
                self.visualize = True
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "v" key ...')
                self.get_logger().info("Opening the visualization")
            
            
            elif key == keyboard.Key.esc:
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the escape key ...')
                self.get_logger().info("Closing the visualization")

            else:
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('/!\ You did not pressed a known key')
                self.get_logger().info('Press the "s" key to save the image.')
                self.get_logger().info('Press the "v" key to visualize the image.')
                self.get_logger().info('Press the Escape arrow key to close your visualisation.')
        except:
            self.get_logger().info("Press 'm' or 'v'")


    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.visualize == True:
            cv2.imshow('Image avec points d\'intérêt', cv_image)
            cv2.waitKey(1)
        if self.save == True:
            cv2.imwrite(self.path, cv_image) #save in your ros2_workspace

def main(args=None):
    rclpy.init(args=args)
    save_image = SaveImage()
    rclpy.spin(save_image)
    save_image.destroy_node()
    rclpy.shutdown()
    save_image.listener.stop()


if __name__ == '__main__':
    main()

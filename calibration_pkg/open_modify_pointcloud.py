import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from pynput import keyboard

import open3d as o3d
import numpy as np
from typing import List

class OpenModifyPointcloud(Node):
    def __init__(self):
        super().__init__('open_modify_pointcloud_node')         
        self.visualize = False # boolean : do the user wants to view the pointcloud ?
        self.modify = False
        self.first_time = True
        self.timer = self.create_timer(0.1, self.main)
        self.print_keypoints = False
        
        # PARAMETERS
        self.vis = o3d.visualization.Visualizer()
        self.declare_parameter("xmin", -10.)
        self.declare_parameter("xmax", +10.)
        self.declare_parameter("ymin", -10.)
        self.declare_parameter("ymax", +10.)
        self.declare_parameter("zmin", -2.)
        self.declare_parameter("zmax", +2.)
        self.declare_parameter("shift", +1.0)
        

        self.xmin  = self.get_parameter("xmin").value
        self.xmax  = self.get_parameter("xmax").value
        self.ymin  = self.get_parameter("ymin").value
        self.ymax  = self.get_parameter("ymax").value
        self.zmin  = self.get_parameter("zmin").value
        self.zmax  = self.get_parameter("zmax").value
        self.shift = self.get_parameter("shift").value
        self.add_on_set_parameters_callback(self.param_callback)

        # Pointcloud
        self.path = "/home/premove/ros2_ws/save_pointcloud/pointcloud_1.pcd"
        self.pointcloud = o3d.io.read_point_cloud(self.path)

        # KEYBOARD
        self.listener = keyboard.Listener(on_press=self.on_key)
        self.listener.start()
        self.get_logger().info('---------------------------------------------')
        self.get_logger().info('/!\ You did not pressed a known key')
        self.get_logger().info('Press the "m" key to modify the pointcloud.')
        self.get_logger().info('Press the "v" key to visualize the pointcloud.')
        self.get_logger().info('Press the Escape arrow key to close your visualisation.')

    def on_key(self,key):
        try:
            self.shift = self.get_parameter("shift").value
            
            self.get_logger().info('---------------------------------')
            self.get_logger().info("You pressed the '"+str(key.char)+"' key")
            self.get_logger().info('----------- shift: '+str(self.shift))
            if key.char == "m":
                self.modify = True
                self.get_logger().info("modifying the pointcloud")

            elif key.char == "v":
                self.visualize = True
                self.get_logger().info("Opening the visualization")
            
            elif key.char == "p":
                for k in range(len(self.pointcloud.points)):
                    x,y,z = self.pointcloud.points[k]
                    self.get_logger().info("(" + str(np.around(x, decimals=2)) + "," + str(np.around(y, decimals=2)) + ","+str(np.around(z, decimals=2)) + ")")

                self.get_logger().info("Printing the keypoints")
            
            #Xmin
            elif key.char == "l": #Xmin goes <--
                self.get_logger().info("Xmin decreases")
                self.xmin -= self.shift
                self.get_logger().info("Xmin = " + str(self.xmin))

            elif key.char == "s": #Xmin goes -->
                
                if self.xmin + self.shift < self.xmax:
                    self.get_logger().info("Xmin increases")
                    self.xmin += self.shift
                else:
                    self.get_logger().info("Xmin cannot increase because of Xmax")
                self.get_logger().info("Xmin = " + str(self.xmin))

            #Xmax
            elif key.char == "w": #Xmax goes <--
                if self.xmax - self.shift > self.xmin:
                    self.get_logger().info("Xmax decreases")
                    self.xmax -= self.shift
                else:
                    self.get_logger().info("Xmax cannot decrease because of Xmin")
                self.get_logger().info("Xmax = " + str(self.xmax))

            elif key.char == "o": #Xmax goes -->
                self.get_logger().info("Xmax increases")
                self.xmax += self.shift
                self.get_logger().info("Xmax = " + str(self.xmax))
            #Ymin
            elif key.char == "d":
                self.get_logger().info("Ymin increases")
                if self.ymin +self.shift < self.ymax:
                    self.ymin += self.shift
                else:
                    self.get_logger().info("Ymin cannot increase")
                self.get_logger().info("Ymin = " + str(self.ymin))

            elif key.char == "ñ":
                self.get_logger().info("Ymin decreases")
                self.ymin -= self.shift
                self.get_logger().info("Ymin = " + str(self.ymin))

            #Ymax
            elif key.char == "k":
                self.get_logger().info("Ymax increases")
                self.ymax += self.shift
                self.get_logger().info("Ymax = " + str(self.ymax))

            elif key.char == "a":
                self.get_logger().info("Ymax decreases")
                if self.ymax - self.shift > self.ymin:
                    self.ymax -= self.shift
                else:
                    self.get_logger().info("Ymax cannot decrease because of Ymin")
                self.get_logger().info("Ymax = " + str(self.ymax))
            
            #Zmin
            elif key.char == "g":
                self.get_logger().info("Zmin increases")
                if self.zmin +self.shift < self.zmax:
                    self.zmin += self.shift
                else:
                    self.get_logger().info("zmin cannot increase")
                self.get_logger().info("zmin = " + str(self.zmin))

            elif key.char == "b":
                self.get_logger().info("zmin decreases")
                self.zmin -= self.shift
                self.get_logger().info("zmin = " + str(self.zmin))

            #Zmax
            elif key.char == "r":
                self.get_logger().info("zmax increases")
                self.zmax += self.shift
                self.get_logger().info("zmax = " + str(self.zmax))

            elif key.char == "f":
                self.get_logger().info("zmax decreases")
                if self.zmax - self.shift > self.zmin:
                    self.zmax -= self.shift
                else:
                    self.get_logger().info("zmax cannot decrease because of zmin")
                self.get_logger().info("zmax = " + str(self.zmax))

            elif key == keyboard.Key.esc:
                self.get_logger().info("Closing the visualization")

            else:
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('/!\ You did not pressed a known key')
                self.get_logger().info('Press the "m" key to modify the pointcloud.')
                self.get_logger().info('Press the "v" key to visualize the pointcloud.')
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
            original_pointcloud = o3d.io.read_point_cloud(self.path)
            L = []
            for k in range(len(original_pointcloud.points)):
                x,y,z = original_pointcloud.points[k]
                if self.xmin < x < self.xmax:
                    if self.ymin < y < self.ymax:
                        if self.zmin < z < self.zmax:
                            L.append([x,y,z])
            self.pointcloud.points = o3d.utility.Vector3dVector(np.array(L))
            
        if self.visualize == True:
            if self.first_time == True:
                self.vis.create_window()
                self.first_time = False
                self.vis.add_geometry(self.pointcloud)
            else:   
                self.vis.update_geometry(self.pointcloud)
                self.vis.poll_events()
                self.vis.update_renderer()
        

def main(args=None):
    rclpy.init(args=args)
    calibration = OpenModifyPointcloud()
    rclpy.spin(calibration)
    calibration.vis.destroy_window()
    calibration.destroy_node()
    rclpy.shutdown()
    calibration.listener.stop()


if __name__ == '__main__':
    main()

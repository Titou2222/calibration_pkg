import rclpy
from rclpy.node import Node

from pynput import keyboard

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import open3d as o3d

class SavePointcloud(Node):
    def __init__(self):
        super().__init__('save_pointcloud_node') 
 
        self.lidar_subscriber = self.create_subscription(PointCloud2, '/rslidar_points', self.callback, 10)
        
        self.save = False # boolean : do the user wants to save the pointcloud ?
        self.visualize = False # boolean : do the user wants to view the pointcloud ?
        self.num = 0 #number of pointcloud saves

        self.pointcloud = o3d.geometry.PointCloud()

        # KEYBOARD
        self.listener = keyboard.Listener(on_press=self.on_key)
        self.listener.start()
        self.get_logger().info('---------------------------------------------')
        self.get_logger().info('Press the "s" key to save the pointcloud.')
        self.get_logger().info('Press the "v" key to visualize the pointcloud.')
        self.get_logger().info('Press the Escape arrow key to close your visualisation.')
        
        
        self.path = "/home/premove/ros2_ws/save_pointcloud/pointcloud_" + str(self.num) + ".pcd"

        self.vis = o3d.visualization.Visualizer()
        self.first_time = True

    def on_key(self,key):
        try:
            if key.char == "s":
                self.save = True
                self.num +=1
                self.path = "/home/premove/ros2_ws/save_pointcloud/pointcloud_" + str(self.num) + ".pcd"
                self.get_logger().info('---------------------------------------------')
                self.get_logger().info('You pressed the "s" key ...')
                self.get_logger().info("Pointcloud saved at " + self.path + "/home/premove/ros2_ws/save_pointcloud/pointcloud_" + str(self.num) + ".pcd")

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
                self.get_logger().info('Press the "s" key to save the pointcloud.')
                self.get_logger().info('Press the "v" key to visualize the pointcloud.')
                self.get_logger().info('Press the Escape arrow key to close your visualisation.')
        except:
            self.get_logger().info("Press 's' or 'v'")
    

    def callback(self, msg):
        lidar_points_np = point_cloud2.read_points_numpy(msg, skip_nans=True)[:, :3]
        self.pointcloud.points = o3d.utility.Vector3dVector(lidar_points_np)
        
        if self.visualize == True:
            if self.first_time == True:
                self.vis.create_window()
                self.first_time = False
                self.vis.add_geometry(self.pointcloud)
            else:   
                self.vis.update_geometry(self.pointcloud)
                self.vis.poll_events()
                self.vis.update_renderer()
        if self.save == True:
            o3d.io.write_point_cloud(self.path, self.pointcloud) #save in your ros2_workspace

def main(args=None):
    rclpy.init(args=args)
    save_pointcloud = SavePointcloud()
    rclpy.spin(save_pointcloud)
    save_pointcloud.destroy_node()
    rclpy.shutdown()
    save_pointcloud.listener.stop()


if __name__ == '__main__':
    main()

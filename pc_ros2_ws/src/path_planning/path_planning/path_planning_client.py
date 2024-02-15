import sys

import rclpy
from rclpy.node import Node

import numpy as np
import cv2
import matplotlib.pyplot as plt

from interfaces.msg import Pose2D
from interfaces.srv import PathPlanning

class PathPlanningClient(Node):

    def __init__(self):

        super().__init__('path_planning_client')

        self.declare_parameter("map_path", "")

        self.path_planning_client = self.create_client(PathPlanning, "path_planning")

        while not self.path_planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.request = PathPlanning.Request()

    def send_request(self, start, end):
    
        start_pose = Pose2D()
        end_pose = Pose2D()

        start_pose.x = start[0]
        start_pose.y = start[1]
        start_pose.theta = start[2]

        end_pose.x = end[0]
        end_pose.y = end[1]
        end_pose.theta = end[2]

        self.request.start = start_pose
        self.request.end = end_pose
        
        self.future = self.path_planning_client.call_async(self.request)
        
        self.future.add_done_callback(self.get_response)

    def get_response(self, future):

        response = future.result()
        
        path = np.zeros((len(response.path),3))

        for i, p in enumerate(response.path):

            path[i,:] = [p.x, p.y, p.theta]

        self.get_logger().info(str(path))

        map_path = self.get_parameter('map_path').get_parameter_value().string_value

        map = cv2.imread(map_path)
        map = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
        map = cv2.threshold(map, 128, 1, cv2.THRESH_BINARY_INV)[1]

        plt.imshow(map, cmap='binary')
        plt.plot(map.shape[1]/2 - path[:,1]*10, map.shape[0]/2 - path[:,0]*10)
        plt.grid()
        plt.show()




def main(args=None):

    rclpy.init(args=args)

    path_planning_client = PathPlanningClient()
    
    path_planning_client.send_request((float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])), (float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])))
    
    rclpy.spin(path_planning_client)

    path_planning_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
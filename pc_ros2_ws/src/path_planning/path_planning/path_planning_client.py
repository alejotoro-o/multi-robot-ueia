import sys

import rclpy
from rclpy.node import Node

import numpy as np
import matplotlib.pyplot as plt

from interfaces.msg import Pose2D
from interfaces.srv import PathPlanning

class PathPlanningClient(Node):

    def __init__(self):

        super().__init__('path_planning_client')

        self.declare_parameter("map_path")

        self.path_planning_client = self.create_client(PathPlanning, "path_planning")

        while not self.path_planning_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.request = PathPlanning.Request()

    def send_request(self, start, end):

        map_path = self.get_parameter('map_path').get_parameter_value().string_value
    
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

        response = self.path_planning_client.call(self.request)

        path = np.zeros((len(response.path),3))

        for i, p in enumerate(response.path):

            path[i,:] = [p.x, p.y, p.theta]

        
        plt.imshow(map_path)
        plt.plot(path[:,1], path[:,0])

        rclpy.shutdown()




def main(args=None):

    rclpy.init(args=args)

    path_planning_client = PathPlanningClient()

    path_planning_client.send_request((float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])), (float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6])))

    rclpy.spin(path_planning_client)

    path_planning_client.destroy_node()


if __name__ == '__main__':
    main()
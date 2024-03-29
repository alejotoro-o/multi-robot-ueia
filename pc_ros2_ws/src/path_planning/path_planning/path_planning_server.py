import rclpy
from rclpy.node import Node

import cv2
from .a_star import astar
import numpy as np

from interfaces.msg import Pose2D
from interfaces.srv import PathPlanning

class PathPlanningServer(Node):

    def __init__(self):

        super().__init__('path_planning_server')

        self.declare_parameter("map_path", "")
        
        self.path_planning_srv = self.create_service(PathPlanning, "path_planning", self.plan_path_callback)

    def plan_path_callback(self, request, response):

        map_path = self.get_parameter('map_path').get_parameter_value().string_value
        map = cv2.imread(map_path)
        map = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
        map = cv2.threshold(map, 128, 1, cv2.THRESH_BINARY_INV)[1]

        start = (int(map.shape[0]/2 - round(request.start.x*10)), int(map.shape[1]/2 - round(request.start.y*10)))
        end = (int(map.shape[0]/2 - round(request.end.x*10)), int(map.shape[1]/2 - round(request.end.y*10)))

        if map[end[0]][end[1]] == 1:
            response.path = []
            return response
        
        path = astar(map, start, end)

        theta_error = request.end.theta - request.start.theta
        if theta_error > np.pi:
            theta_error += -2*np.pi
        elif theta_error < -np.pi:
            theta_error += 2*np.pi

        orient_error = np.linspace(theta_error, 0, len(path))

        response.path = []

        for i, p in enumerate(path):

            point = Pose2D()
            point.x = (map.shape[0]/2 - p[0])*0.1
            point.y = (map.shape[1]/2 - p[1])*0.1
            point.theta = request.end.theta - orient_error[i]
            point.theta = ((point.theta + np.pi)%(2*np.pi)) - np.pi

            response.path.append(point)
        
        return response



def main(args=None):

    rclpy.init(args=args)

    path_planning_server = PathPlanningServer()

    rclpy.spin(path_planning_server)

    path_planning_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
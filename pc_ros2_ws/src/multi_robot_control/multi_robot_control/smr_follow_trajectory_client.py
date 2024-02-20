import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from interfaces.msg import Pose2D, Obstacle
from interfaces.action import FollowTrajectory
from interfaces.srv import ObsPathPlanning


class SMRFollowTrajectoryActionClient(Node):

    def __init__(self):

        super().__init__('smr_follow_trajectory_client')

        self.pose_subscription = self.create_subscription(Pose, 'robot1/pose', self.get_robot1_pose_callback, 1)
        self.pose_subscription = self.create_subscription(Pose, 'robot2/pose', self.get_robot2_pose_callback, 1)
        self.q1 = Pose()
        self.q2 = Pose()

        self.path_planning_client = self.create_client(ObsPathPlanning, "path_planning")

        self._action_client_robot1 = ActionClient(self, FollowTrajectory, 'robot1/follow_trajectory')
        self._action_client_robot2 = ActionClient(self, FollowTrajectory, 'robot2/follow_trajectory')
        self.goal_handle = None

        self.executing = False

    def get_robot1_pose_callback(self, pose):

        self.q1 = pose

    def get_robot2_pose_callback(self, pose):

        self.q2 = pose


    def send_goal(self, x, y, theta1, theta2, time1, time2):

        goal_msg = FollowTrajectory.Goal()

        ## Robot 1
        r1 = R.from_quat([self.q1.orientation.x, self.q1.orientation.y, self.q1.orientation.z, self.q1.orientation.w])    
        q1_theta = r1.as_rotvec()[-1]

        start_robot1 = (self.q1.position.x, self.q1.position.y, q1_theta)
        end_robot1 = (x, y - 0.2, theta1)

        ## Robot 2
        r2 = R.from_quat([self.q2.orientation.x, self.q2.orientation.y, self.q2.orientation.z, self.q2.orientation.w])    
        q2_theta = r2.as_rotvec()[-1]

        start_robot2 = (self.q2.position.x, self.q2.position.y, q2_theta)
        end_robot2 = (x, y + 0.2, theta2)

        self.path_planning_client.wait_for_service()
        
        ## Robot 1 path planning
        self.get_logger().info("Planning robot 1 path...")
        goal_msg.path  = self.get_path(start_robot1, end_robot1, (start_robot2[0],start_robot2[1],0.2))
        goal_msg.time = time1

        self._action_client_robot1.wait_for_server()

        self._send_goal_future_robot1 = self._action_client_robot1.send_goal_async(goal_msg)

        self._send_goal_future_robot1.add_done_callback(self.goal_response_callback)
        
        self.executing = True

        while self.executing:
            pass

        ## Robot 2 path planning
        self.get_logger().info("Planning robot 2 path...")
        goal_msg.path  = self.get_path(start_robot2, end_robot2, (end_robot1[0],end_robot1[1],0.2))
        goal_msg.time = time2

        self._action_client_robot2.wait_for_server()

        self._send_goal_future_robot2 = self._action_client_robot2.send_goal_async(goal_msg)

        self._send_goal_future_robot2.add_done_callback(self.goal_response_callback)

    def get_path(self, start, end, obs):

        request = ObsPathPlanning.Request()

        start_pose = Pose2D()
        end_pose = Pose2D()
        obstacle = Obstacle()

        start_pose.x = start[0]
        start_pose.y = start[1]
        start_pose.theta = start[2]

        end_pose.x = end[0]
        end_pose.y = end[1]
        end_pose.theta = end[2]

        obstacle.x = obs[0]
        obstacle.y = obs[1]
        obstacle.radius = obs[2]

        request.start = start_pose
        request.end = end_pose
        request.obstacles = [obstacle]
        
        future = self.path_planning_client.call_async(request)
        
        while not future.done():
            pass

        return future.result().path
      

    def cancel_goal(self):

        if self.goal_handle:
            self.get_logger().info('Canceling goal')
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')


    def goal_response_callback(self, future):

        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        self.executing = False

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

def control_loop(node):

    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    while 1:
        
        goal = input()

        if goal == 'c':
            node.cancel_goal()
            continue
        elif goal == 'q':
            break

        x, y, theta1, theta2, time1, time2 = goal.split(' ')

        node.send_goal(float(x), float(y), float(theta1), float(theta2), float(time1), float(time2))


def main(args=None):

    rclpy.init(args=args)

    action_client = SMRFollowTrajectoryActionClient()

    control_loop(action_client)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
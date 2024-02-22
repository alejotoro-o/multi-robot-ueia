import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import numpy as np
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Pose
from interfaces.msg import Pose2D, Obstacle
from interfaces.action import ApproachObject, FollowTrajectory
from interfaces.srv import ObsPathPlanning


class ApproachObjectServer(Node):

    def __init__(self):

        super().__init__('approach_object_server')

        self.pose_subscription = self.create_subscription(Pose, 'robot1/pose', self.get_robot1_pose_callback, 1)
        self.pose_subscription = self.create_subscription(Pose, 'robot2/pose', self.get_robot2_pose_callback, 1)
        self.q1 = Pose()
        self.q2 = Pose()

        self.path_planning_client = self.create_client(ObsPathPlanning, "path_planning")

        self._action_server = ActionServer(self, ApproachObject, 'approach_object',
                                           self.approach_object_callback,
                                           cancel_callback=self.cancel_approach_object_callback)
        self.approach_object_goal_handle = None

        self._action_client_robot1 = ActionClient(self, FollowTrajectory, 'robot1/follow_trajectory')
        self._action_client_robot2 = ActionClient(self, FollowTrajectory, 'robot2/follow_trajectory')
        self.goal_handle = None

        self.executing = False


    ###############################
    ## APPROACH OBJECT CALLBACKS ##
    ###############################
        
    def approach_object_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')

        q1_goal, q2_goal = self.get_q_goals(goal_handle.request.object_pose, goal_handle.request.radius)

        self.send_goal(q1_goal, q2_goal, 10.0, goal_handle.request)

        goal_handle.succeed()

        goal_result = ApproachObject.Result()

        goal_result.result = "Approach complete"
        self.get_logger().info(goal_result.result)

        return goal_result


    def cancel_approach_object_callback(self, goal_handle):

        self.get_logger().info('Received cancel request')
 
        self.cancel_goal()

        return CancelResponse.ACCEPT
    
    def get_q_goals(self, object_pose, object_radius):

        q1_x = object_pose.x + object_radius*np.sin(object_pose.theta)
        q1_y = object_pose.y - object_radius*np.cos(object_pose.theta)
        q1_theta = np.pi/2 + object_pose.theta

        q2_x = object_pose.x - object_radius*np.sin(object_pose.theta)
        q2_y = object_pose.y + object_radius*np.cos(object_pose.theta)
        q2_theta = object_pose.theta - np.pi/2 

        return (q1_x, q1_y, q1_theta), (q2_x, q2_y, q2_theta)


    ########################################
    ## FOLLOW TRAJECTORY SERVER CALLBACKS ##
    ########################################
        
    def get_robot1_pose_callback(self, pose):

        self.q1 = pose

    def get_robot2_pose_callback(self, pose):

        self.q2 = pose


    def send_goal(self, q1_goal, q2_goal, time, object):

        goal_msg = FollowTrajectory.Goal()

        ## Robot 1
        r1 = R.from_quat([self.q1.orientation.x, self.q1.orientation.y, self.q1.orientation.z, self.q1.orientation.w])    
        q1_theta = r1.as_rotvec()[-1]

        start_robot1 = (self.q1.position.x, self.q1.position.y, q1_theta)
        end_robot1 = q1_goal

        ## Robot 2
        r2 = R.from_quat([self.q2.orientation.x, self.q2.orientation.y, self.q2.orientation.z, self.q2.orientation.w])    
        q2_theta = r2.as_rotvec()[-1]

        start_robot2 = (self.q2.position.x, self.q2.position.y, q2_theta)
        end_robot2 = q2_goal

        self.path_planning_client.wait_for_service()
        
        ## Robot 1 path planning
        self.get_logger().info("Planning robot 1 path...")
        goal_msg.path  = self.get_path(start_robot1, end_robot1,
                                       [
                                           (start_robot2[0],start_robot2[1],0.2),
                                           (object.object_pose.x, object.object_pose.y, object.radius),
                                       ])
        goal_msg.time = time

        self._action_client_robot1.wait_for_server()

        self._send_goal_future_robot1 = self._action_client_robot1.send_goal_async(goal_msg)

        self._send_goal_future_robot1.add_done_callback(self.goal_response_callback)
        
        self.executing = True

        while self.executing:
            pass

        ## Robot 2 path planning
        self.get_logger().info("Planning robot 2 path...")
        goal_msg.path  = self.get_path(start_robot2, end_robot2,
                                       [
                                           (end_robot1[0],end_robot1[1],0.2),
                                           (object.object_pose.x, object.object_pose.y, object.radius),
                                       ])

        self._action_client_robot2.wait_for_server()

        self._send_goal_future_robot2 = self._action_client_robot2.send_goal_async(goal_msg)

        self._send_goal_future_robot2.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass


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

        request.obstacles = []

        for o in obs:

            obstacle.x = o[0]
            obstacle.y = o[1]
            obstacle.radius = o[2]

            request.obstacles.append(obstacle)

        request.start = start_pose
        request.end = end_pose

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

def main(args=None):

    rclpy.init(args=args)

    approach_object_server = ApproachObjectServer()

    rclpy.spin(approach_object_server, executor=MultiThreadedExecutor())

    approach_object_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from interfaces.action import NavToPose, FollowPath


class FollowPathServer(Node):

    def __init__(self):

        super().__init__('follow_path_server')

        self._follow_path_server = ActionServer(self, FollowPath,
                                                      'follow_path',
                                                      self.follow_path_callback)
        self._nav_to_pose_client = ActionClient(self, NavToPose, 'nav_to_pose')

        self.follow_path_goal_handle = None
        self.nav_to_pose_goal_handle = None

        self.goal_result = FollowPath.Result()
        
    ##################################
    ## FOLLOW PATH SERVER CALLBACKS ##
    ##################################
        
    async def follow_path_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')

        self.follow_path_goal_handle = goal_handle

        ################
        for waypoint in self.follow_path_goal_handle.request.path:

            goal_msg = NavToPose.Goal()
            goal_msg.pose.x = waypoint.x
            goal_msg.pose.y = waypoint.y
            goal_msg.pose.theta = waypoint.theta

            self._nav_to_pose_client.wait_for_server()
            self._send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
            self._send_goal_future.add_done_callback(self.goal_response_callback)

            self.navigating_to_waypoint = True

            while self.navigating_to_waypoint:
                pass
            
        ################

        self.follow_path_goal_handle.succeed()

        self.goal_result.result = "Path complete"
        self.get_logger().info(self.goal_result.result)

        self.follow_path_goal_handle = None

        return self.goal_result

    def cancel_callback(self, goal_handle):

        self.get_logger().info('Received cancel request')
 
        self.cancel_nav_to_pose_goal()

        return CancelResponse.ACCEPT

    ##################################
    ## NAV TO POSE CLIENT CALLBACKS ##
    ##################################

    def send_goal(self, x, y, theta):

        goal_msg = NavToPose.Goal()
        goal_msg.pose.x = x
        goal_msg.pose.y = y
        goal_msg.pose.theta = theta

        self._nav_to_pose_client.wait_for_server()

        self._send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):

        self.nav_to_pose_goal_handle = future.result()
        if not self.nav_to_pose_goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = self.nav_to_pose_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        self.navigating_to_waypoint = False

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

    def cancel_nav_to_pose_goal(self):

        if self.nav_to_pose_goal_handle:
            self.get_logger().info('Canceling goal')
            future = self.nav_to_pose_goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)

    def cancel_done(self, future):
        
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')


def main(args=None):

    rclpy.init(args=args)

    follow_path_server = FollowPathServer()

    rclpy.spin(follow_path_server, executor=MultiThreadedExecutor())

    follow_path_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
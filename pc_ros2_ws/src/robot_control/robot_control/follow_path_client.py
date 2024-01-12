import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from interfaces.msg import Pose2D
from interfaces.action import FollowPath


class FollowPathActionClient(Node):

    def __init__(self):

        super().__init__('follow_path_client')
        self._action_client = ActionClient(self, FollowPath, 'follow_path')
        self.goal_handle = None

    def format_goal(self, path):

        path_msg = []

        for waypoint in path:

            waypoint_msg = Pose2D()

            waypoint_msg.x = float(waypoint[0])
            waypoint_msg.y = float(waypoint[1])
            waypoint_msg.theta = float(waypoint[2])

            path_msg.append(waypoint_msg)

        return path_msg


    def send_goal(self, path):

        goal_msg = FollowPath.Goal()
        goal_msg.path = self.format_goal(path)

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


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

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

def control_loop(node):

    thread = threading.Thread(target=rclpy.spin, args=(node,))
    thread.start()

    while 1:

        goal = input()

        if goal == '1':
            goal = [[0, -0.5, 0],
                    [0.5, -0.5, 1.57],
                    [0.5, 0.5, 3.14],
                    [-0.5, 0.5, -1.57],
                    [-0.5, -0.5, 0],
                    [0, -0.5, 0],
                    [0, 0, 0]]
        elif goal == 'c':
            node.cancel_goal()
            continue
        elif goal == 'q':
            break

        node.send_goal(goal)


def main(args=None):

    rclpy.init(args=args)

    action_client = FollowPathActionClient()

    control_loop(action_client)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
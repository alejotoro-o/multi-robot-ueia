
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from control import dlqr

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from interfaces.action import NavToPose

class LQRController(Node):

    def __init__(self):

        super().__init__('lqr_controller')

        # Controller parameters
        # self.declare_parameter('wheel_radius', 0.0325)
        # self.declare_parameter('lx', 0.0845)
        # self.declare_parameter('ly', 0.08)
        # self.declare_parameter('Q', 0.1*[1., 0., 0., 0., 1., 0., 0., 0., 1.])
        # self.declare_parameter('R', 0.1*[1., 0., 0., 0., 1., 0., 0., 0., 1.])

        # Controller gains
        Q = 0.1*np.identity(3)
        R = 0.1*np.identity(3)
        self.K, _, _ = dlqr(np.identity(3),np.identity(3),Q,R)

        # Jacobian
        # radius = self.get_parameter('wheel_radius').value
        # lx = self.get_parameter('lx').value
        # ly = self.get_parameter('ly').value
        # self.J = (radius/4.0)*np.array([[1, 1, 1, 1],
        #                                 [-1, 1, 1, -1],
        #                                 [-1/(lx+ly), 1/(lx+ly), -1/(lx+ly), 1/(lx+ly)]])

        # Publishers, subscribers and actions
        self._pose_subscription = self.create_subscription(Pose, 'pose', self._control_callback, 1)
        self._cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self._nav_to_pose_action_server = ActionServer(self, NavToPose, 'nav_to_pose', self.nav_to_pose_callback, cancel_callback=self.cancel_callback)
        self.goal_handle = None
        self.goal_result = NavToPose.Result()
        self.feedback_msg = NavToPose.Feedback()

        self.q_goal = np.array([[0,0,0]]).T

    def _control_callback(self, pose):

        r = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
       
        theta = r.as_rotvec()[-1]
        self.q = np.array([[pose.position.x, pose.position.y, theta]]).T

        v = np.dot(self.K, (self.q_goal - self.q))

        rot_m = np.array([[np.cos(theta), -np.sin(theta), 0],
                          [np.sin(theta), np.cos(theta), 0],
                          [0, 0, 1]])
        
        u = np.dot(np.linalg.pinv(rot_m), v).squeeze()

        twist_msg = Twist()

        twist_msg.linear.x = u[0]
        twist_msg.linear.y = u[1]
        twist_msg.angular.z = u[2]

        self._cmd_vel_publisher.publish(twist_msg)
        
    # def accept_nav_to_pose_callback(self, goal_handle):

    #     if self.goal_handle:
            
    #         self.get_logger().info('New goal received, aborting previous goal')
    #         self.goal_handle.abort()
    #         self.get_logger().info(str(self.goal_handle.status))

    #     goal_handle.execute(self.nav_to_pose_callback)

    async def nav_to_pose_callback(self, goal_handle):

        if self.goal_handle:
            
            self.get_logger().info('New goal received, aborting previous goal')
            self.goal_handle.abort()

        self.get_logger().info('Executing goal...')

        self.goal_handle = goal_handle

        x_goal = goal_handle.request.x
        y_goal = goal_handle.request.y
        theta_goal = goal_handle.request.theta

        self.q_goal = np.array([[x_goal, y_goal, theta_goal]]).T

        distance_to_goal = math.sqrt((self.q_goal[0] - self.q[0])**2 + (self.q_goal[1] - self.q[1])**2)
        heading_error = self.q_goal.squeeze()[2] - self.q.squeeze()[2]

        self.feedback_msg.error = [distance_to_goal, heading_error]
        self.get_logger().info('Feedback: {0}'.format(self.feedback_msg.error))

        while (self.feedback_msg.error[0] > 0.01 or abs(self.feedback_msg.error[1]) > 0.01):

            distance_to_goal = math.sqrt((self.q_goal[0] - self.q[0])**2 + (self.q_goal[1] - self.q[1])**2)
            heading_error = self.q_goal.squeeze()[2] - self.q.squeeze()[2]

            self.feedback_msg.error = [distance_to_goal, heading_error]
            self.get_logger().info('Feedback: {0}'.format(self.feedback_msg.error))

            if self.goal_handle.is_cancel_requested:

                self.goal_handle.canceled()
                self.goal_handle = None
                self.goal_result.result = "Goal canceled"
                self.get_logger().info(self.goal_result.result)
                return self.goal_result

            if self.goal_handle != goal_handle:

                self.goal_result.result = "New goal received, aborting previous goal"
                self.get_logger().info(self.goal_result.result)
                return self.goal_result


        self.goal_handle.succeed()

        self.goal_result.result = "Arrived to goal"
        self.get_logger().info(self.goal_result.result)

        self.goal_handle = None

        return self.goal_result
    
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')

        if self.goal_handle:

            self.q_goal = self.q

        return CancelResponse.ACCEPT



def main(args=None):

    rclpy.init(args=args)

    lqr_controller_node = LQRController()

    rclpy.spin(lqr_controller_node, executor=MultiThreadedExecutor())

    lqr_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
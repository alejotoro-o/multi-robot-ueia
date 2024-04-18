
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from control import dlqr

import rclpy
from typing import Optional
from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from interfaces.action import NavToPose

class LQRController(Node):

    def __init__(self):

        super().__init__('lc_lqr_controller')

        # Controller parameters
        self.declare_parameter("initial_pose", [0.0,0.0,0.0])
        # self.declare_parameter('wheel_radius', 0.0325)
        # self.declare_parameter('lx', 0.0845)
        # self.declare_parameter('ly', 0.08)
        self.declare_parameter('Q_factor', 0.1)
        self.declare_parameter('R_factor', 0.1)

        # Controller gains
        Q_factor = self.get_parameter('Q_factor').get_parameter_value().double_value
        R_factor = self.get_parameter('R_factor').get_parameter_value().double_value
        Q = Q_factor*np.identity(3)
        R = R_factor*np.identity(3)
        self.K, _, _ = dlqr(np.identity(3),np.identity(3),Q,R)

        # Jacobian
        # radius = self.get_parameter('wheel_radius').value
        # lx = self.get_parameter('lx').value
        # ly = self.get_parameter('ly').value
        # self.J = (radius/4.0)*np.array([[1, 1, 1, 1],
        #                                 [-1, 1, 1, -1],
        #                                 [-1/(lx+ly), 1/(lx+ly), -1/(lx+ly), 1/(lx+ly)]])

        # Publishers, subscribers and actions
        self._pose_subscription = self.create_subscription(Pose, 'pose', self._control_callback, 10)
        self._cmd_vel_publisher: Optional[Publisher] = None

        self._nav_to_pose_action_server = ActionServer(self, NavToPose,
                                                       'nav_to_pose',
                                                       self.nav_to_pose_callback,
                                                       cancel_callback=self.cancel_callback)
        self.goal_handle = None
        self.goal_result = NavToPose.Result()
        self.feedback_msg = NavToPose.Feedback()

        self.q_goal = np.array(self.get_parameter('initial_pose').get_parameter_value().double_array_value).reshape((3,1))

    def _control_callback(self, pose):

        if pose.orientation.x == 0 and pose.orientation.y == 0 and pose.orientation.z == 0 and pose.orientation.w == 0:
            theta = self.q[2,0]
        else:
            r = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            theta = r.as_rotvec()[-1]

            self.q = np.array([[pose.position.x, pose.position.y, theta]]).T

        pose_error = np.zeros((3,1))
        pose_error[:-1] = self.q_goal[:-1] - self.q[:-1]

        # Shortest rotation
        pose_error[-1] = self.q_goal.squeeze()[2] - theta
        if pose_error[-1] > np.pi:
            pose_error[-1] += -2*np.pi
        elif pose_error[-1] < -np.pi:
            pose_error[-1] += 2*np.pi

        # pose_error = (self.q_goal - self.q)
        v = np.dot(self.K, pose_error)

        rot_m = np.array([[np.cos(theta), -np.sin(theta), 0],
                          [np.sin(theta), np.cos(theta), 0],
                          [0, 0, 1]])
        
        u = np.dot(np.linalg.pinv(rot_m), v).squeeze()

        twist_msg = Twist()

        twist_msg.linear.x = u[0]
        twist_msg.linear.y = u[1]
        twist_msg.angular.z = u[2]

        if self._cmd_vel_publisher is not None:
            self._cmd_vel_publisher.publish(twist_msg)
        

    async def nav_to_pose_callback(self, goal_handle):

        if self.goal_handle:
            
            self.get_logger().info('New goal received, aborting previous goal')
            self.goal_handle.abort()

        self.get_logger().info('Executing goal...')

        self.goal_handle = goal_handle

        x_goal = goal_handle.request.pose.x
        y_goal = goal_handle.request.pose.y
        theta_goal = goal_handle.request.pose.theta

        # Normalize goal angle
        theta_goal = ((theta_goal + np.pi)%(2*np.pi)) - np.pi

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
    

    ##########################
    ## LifeCycle Callabacks ##
    ##########################

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Configure the node, after a configuring transition is requested.

        on_configure callback is being called when the lifecycle node
        enters the "configuring" state.
        
        :return: The state machine either invokes a transition to the "inactive" state or stays
        in "unconfigured" depending on the return value.
        TransitionCallbackReturn.SUCCESS transitions to "inactive".
        TransitionCallbackReturn.FAILURE transitions to "unconfigured".
        TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self._cmd_vel_publisher = self.create_lifecycle_publisher(Twist, "cmd_vel", 10)

        self.get_logger().info("on_configure() is called.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Differently to rclcpp, a lifecycle publisher transitions automatically between the inactive and
        # enabled state and viceversa.
        # For that reason, we only need to write an on_configure() and on_cleanup() callbacks, and we don't
        # need to write on_activate()/on_deactivate() callbacks.

        # Log, only for demo purposes
        self.get_logger().info("on_activate() is called.")

        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        return super().on_activate(state)
  
    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info("on_deactivate() is called.")
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Cleanup the node, after a cleaning-up transition is requested.

        on_cleanup callback is being called when the lifecycle node
        enters the "cleaning up" state.
        
        :return: The state machine either invokes a transition to the "unconfigured" state or stays
        in "inactive" depending on the return value.
        TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
        TransitionCallbackReturn.FAILURE transitions to "inactive".
        TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_publisher(self._cmd_vel_publisher)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Shutdown the node, after a shutting-down transition is requested.

        on_shutdown callback is being called when the lifecycle node
        enters the "shutting down" state.
        
        :return: The state machine either invokes a transition to the "finalized" state or stays
        in the current state depending on the return value.
        TransitionCallbackReturn.SUCCESS transitions to "unconfigured".
        TransitionCallbackReturn.FAILURE transitions to "inactive".
        TransitionCallbackReturn.ERROR or any uncaught exceptions to "errorprocessing"
        """
        self.destroy_publisher(self._cmd_vel_publisher)

        self.get_logger().info('on_shutdown() is called.')
        return TransitionCallbackReturn.SUCCESS


def main(args=None):

    rclpy.init(args=args)

    lqr_controller_node = LQRController()

    rclpy.spin(lqr_controller_node, executor=MultiThreadedExecutor())

    lqr_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
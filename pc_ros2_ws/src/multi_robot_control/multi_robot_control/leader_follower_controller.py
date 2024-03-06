import numpy as np
from scipy.spatial.transform import Rotation as R
from control import dlqr

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class LeaderFollowerController(Node):

    def __init__(self):

        super().__init__('leader_follower_controller')

        # Controller parameters
        self.declare_parameter("robot1_initial_pose", [0.0,0.0,0.0])
        self.declare_parameter("robot2_initial_pose", [0.0,0.0,0.0])
        self.declare_parameter("d_goal", 0.1)
        self.declare_parameter('alpha_goal', 0.0)
        self.declare_parameter('theta_f_goal', 0.0)
        self.declare_parameter("K", [1.0,1.0,1.0])

        # Controller gains
        self.d_goal = self.get_parameter('d_goal').get_parameter_value().double_value
        self.alpha_goal = self.get_parameter('alpha_goal').get_parameter_value().double_value
        self.theta_f_goal = self.get_parameter('theta_f_goal').get_parameter_value().double_value
        self.K = np.array(self.get_parameter('K').get_parameter_value().double_array_value)

        self.q1 = np.array(self.get_parameter('robot1_initial_pose').get_parameter_value().double_array_value)
        self.q2 = np.array(self.get_parameter('robot2_initial_pose').get_parameter_value().double_array_value)
        self.u_l = np.array([[0,0,0]]).T
        self.u_f = np.array([0,0,0])

        # Publishers, subscribers and actions
        self._robot1_pose_subscription = self.create_subscription(Pose, 'robot1/pose', self.__robot1_pose_callback, 1)
        self._robot2_pose_subscription = self.create_subscription(Pose, 'robot2/pose', self.__robot2_pose_callback, 1)
        self._robot1_cmd_vel_subscription = self.create_subscription(Twist, 'robot1/cmd_vel', self.__robot1_cmd_vel_callback, 1)
        self._robot2_cmd_vel_publisher = self.create_publisher(Twist, 'robot2/cmd_vel', 10)

        self.create_timer(0.01, self.__control_callback)

    def __robot1_pose_callback(self, msg):

        if msg.orientation.x == 0 and msg.orientation.y == 0 and msg.orientation.z == 0 and msg.orientation.w == 0:
            theta = self.q1[2]
        else:
            r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            theta = r.as_rotvec()[-1]

            self.q1 = np.array([msg.position.x, msg.position.y, theta])


    def __robot2_pose_callback(self, msg):

        if msg.orientation.x == 0 and msg.orientation.y == 0 and msg.orientation.z == 0 and msg.orientation.w == 0:
            theta = self.q2[2]
        else:
            r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            theta = r.as_rotvec()[-1]

            self.q2 = np.array([msg.position.x, msg.position.y, theta])


    def __robot1_cmd_vel_callback(self, msg):

        v_x = msg.linear.x
        v_y = msg.linear.y
        w_z = msg.angular.z

        self.u_l = np.array([[v_x,v_y,w_z]]).T

    def __control_callback(self):

        self.theta_f_goal = self.alpha_goal = self.q1[2]

        d = np.sqrt((self.q1[0] - self.q2[0])**2 + (self.q1[1] - self.q2[1])**2)
        alpha = self.q2[2] - np.arctan2((self.q1[1] - self.q2[1]), (self.q1[0] - self.q2[0]))
        gamma = self.q1[2] - self.q2[2] + alpha

        A = np.array([[np.cos(gamma),-np.sin(gamma),0],
                      [-(1/d)*np.sin(gamma),-(1/d)*np.cos(gamma),0],
                      [0,0,0]])
        B = np.array([[np.cos(alpha),-np.sin(alpha),0],
                      [-(1/d)*np.sin(alpha),-(1/d)*np.cos(alpha),-1],
                      [0,0,-1]])
        
        #n_dot = np.dot(A, self.u_l) - np.dot(B, self.u_f)

        p_d = np.array([[-self.K[0]*(d - self.d_goal)],
                        [self.u_l[2,0]-self.K[1]*(alpha - self.alpha_goal)],
                        [self.u_l[2,0]-self.K[2]*(self.q2[2] - self.theta_f_goal)]])  
    

        self.u_f = np.dot(np.linalg.inv(B), (np.dot(A, self.u_l) - p_d))
        u_f = self.u_f.squeeze()

        twist_msg = Twist()

        twist_msg.linear.x = float(u_f[0])
        twist_msg.linear.y = float(u_f[1])
        twist_msg.angular.z = float(u_f[2])

        self._robot2_cmd_vel_publisher.publish(twist_msg)

def main(args=None):

    rclpy.init(args=args)

    leader_follower_controller_node = LeaderFollowerController()

    rclpy.spin(leader_follower_controller_node)

    leader_follower_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
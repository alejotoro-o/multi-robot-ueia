import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from control import dlqr

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

class CMDVelTransform(Node):

    def __init__(self):

        super().__init__('cmd_vel_transform')

        self.declare_parameter("d", 0.2)
        self.d = self.get_parameter('d').get_parameter_value().double_value

        self.cmd_vel_subscription = self.create_subscription(Twist, 'transformed_cmd_vel', self.cmd_vel_callback, 1)
        self.pose_subscription = self.create_subscription(Pose, 'pose', self.pose_callback, 1)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.q = np.array([0,0,0])

    def pose_callback(self, msg):

        if msg.orientation.x == 0 and msg.orientation.y == 0 and msg.orientation.z == 0 and msg.orientation.w == 0:
            theta = self.q[2]
        else:
            r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            theta = r.as_rotvec()[-1]

            self.q = np.array([msg.position.x, msg.position.y, theta])

    def cmd_vel_callback(self, msg):

        cmd_vel = Twist()

        rot_m = np.array([[np.cos(self.q[2]), -np.sin(self.q[2]), 0],
                          [np.sin(self.q[2]), np.cos(self.q[2]), 0],
                          [0, 0, 1]])
        
        q_dot_r = np.array([[msg.linear.x,msg.linear.y,msg.angular.z]]).T
        
        q_dot_g = np.dot(rot_m, q_dot_r)

        m = np.array([[np.sin(self.q[2])],
                      [-np.cos(self.q[2])],
                      [0]])

        q_dot_g_trans = q_dot_g + self.d*m*q_dot_g[2]

        q_dot_r_trans = np.dot(np.linalg.inv(rot_m), q_dot_g_trans)
        q_dot_r_trans = q_dot_r_trans.squeeze()

        cmd_vel.linear.x = float(q_dot_r_trans[0])
        cmd_vel.linear.y = float(q_dot_r_trans[1])
        cmd_vel.angular.z = float(q_dot_r_trans[2])

        self.cmd_vel_publisher.publish(cmd_vel)

def main(args=None):

    rclpy.init(args=args)

    cmd_vel_transform_node = CMDVelTransform()

    rclpy.spin(cmd_vel_transform_node)

    cmd_vel_transform_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

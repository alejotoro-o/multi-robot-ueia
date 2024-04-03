import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

import numpy as np
from scipy.spatial.transform import Rotation as R

class ObjectPosePublisher(Node):

    def __init__(self):

        super().__init__('object_pose_publisher')

        self.declare_parameter("d", 0.2)
        self.d = self.get_parameter('d').get_parameter_value().double_value

        self.create_subscription(Pose, 'pose', self.pose_callback, 10)
        self.q = np.array([0,0,0])

        self.object_pose_publisher = self.create_publisher(Pose, 'object/pose', 10)

    def pose_callback(self, msg):

        ## Get Robot Pose
        if msg.orientation.x == 0 and msg.orientation.y == 0 and msg.orientation.z == 0 and msg.orientation.w == 0:
            theta = self.q[2]
        else:
            r = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            theta = r.as_rotvec()[-1]

            self.q = np.array([msg.position.x, msg.position.y, theta])

        ## Estimate Object Pose
        rot_m = np.array([[np.cos(self.q[2]), -np.sin(self.q[2]), 0],
                          [np.sin(self.q[2]), np.cos(self.q[2]), 0],
                          [0, 0, 1]])
        
        q_r = np.dot(np.linalg.inv(rot_m), self.q)
        q_r[0] = q_r[0] + self.d

        q_g = np.dot(rot_m, q_r)

        object_pose = Pose()

        object_pose.position.x = float(q_g[0])
        object_pose.position.y = float(q_g[1])

        r = R.from_rotvec(np.array([0,0,theta]))
        object_pose.orientation.w = r.as_quat()[3]
        object_pose.orientation.x = r.as_quat()[0]
        object_pose.orientation.y = r.as_quat()[1]
        object_pose.orientation.z = r.as_quat()[2]

        self.object_pose_publisher.publish(object_pose)

def main(args=None):

    rclpy.init(args=args)

    object_pose_publisher = ObjectPosePublisher()

    rclpy.spin(object_pose_publisher)

    object_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
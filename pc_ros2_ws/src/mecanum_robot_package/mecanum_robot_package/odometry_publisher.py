import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odometry = Odometry()

        self.last_time = self.get_clock().now()

        self.create_subscription(Twist, 'vel_raw', self.cal_odometry_callback, 1)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 1)
        self.tf_broadcaster = TransformBroadcaster(self)

    def cal_odometry_callback(self, twist):

        self.current_time = self.get_clock().now()
        d_t = (self.current_time - self.last_time).nanoseconds/1e9
        d_x = twist.linear.x*d_t
        d_y = twist.linear.y*d_t
        d_theta = twist.angular.z*d_t

        self.x = self.x + (d_x*np.cos(self.theta) - d_y*np.sin(self.theta))
        self.y = self.y + (d_x*np.sin(self.theta) + d_y*np.sin(self.theta))
        self.theta = self.theta + d_theta

        ## TF transform
        t = TransformStamped()

        t.header.stamp = self.current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        quat = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

        ## Odometry message
        self.odometry.header.stamp = self.current_time.to_msg()
        self.odometry.header.frame_id = 'odom'

        self.odometry.child_frame_id = 'base_link'
        self.odometry.pose.pose.position.x = self.x
        self.odometry.pose.pose.position.y = self.y
        self.odometry.pose.pose.position.z = 0.0

        self.odometry.pose.pose.orientation.x = quat[0]
        self.odometry.pose.pose.orientation.y = quat[1]
        self.odometry.pose.pose.orientation.z = quat[2]
        self.odometry.pose.pose.orientation.w = quat[3]

        self.odom_publisher.publish(self.odometry)

        self.last_time = self.current_time


def main(args=None):

    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)

    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



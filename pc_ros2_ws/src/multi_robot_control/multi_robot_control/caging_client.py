import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from std_msgs.msg import Bool
from interfaces.action import NavToPose, ApproachObject, ControlGripper

import threading
import numpy as np
import time

class CagingClient(Node):

    def __init__(self):

        super().__init__('caging_client')

        self.declare_parameter('object_pose', [0.0,0.0,0.0])
        self.declare_parameter('object_radius', 0.0)

        self.approach_object_client = ActionClient(self, ApproachObject, "approach_object")

        self.robot1_gripper_control_client = ActionClient(self, ControlGripper, "robot1/control_gripper")
        self.robot2_gripper_control_client = ActionClient(self, ControlGripper, "robot2/control_gripper")

        self.robot1_nav_to_pose_client = ActionClient(self, NavToPose, "robot1/nav_to_pose")
        self.robot2_nav_to_pose_client = ActionClient(self, NavToPose, "robot2/nav_to_pose")

        self.executing = False

    def caging(self):

        self.grab_object()
        

    def grab_object(self):

        approach_object_msg = ApproachObject.Goal()

        object_pose = self.get_parameter('object_pose').get_parameter_value().double_array_value
        object_radius = self.get_parameter('object_radius').get_parameter_value().double_value

        approach_object_msg.object_pose.x = object_pose[0]
        approach_object_msg.object_pose.y = object_pose[1]
        approach_object_msg.object_pose.theta = object_pose[2]
        approach_object_msg.radius = object_radius

        self.approach_object_client.wait_for_server()

        approach_object_future = self.approach_object_client.send_goal_async(approach_object_msg)
        approach_object_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass
        
        self.robot1_gripper_control_client.wait_for_server()
        self.robot2_gripper_control_client.wait_for_server()

        gripper_msg = ControlGripper.Goal()
        gripper_msg.open = True

        r1_gripper_future = self.robot1_gripper_control_client.send_goal_async(gripper_msg)
        r1_gripper_future.add_done_callback(self.goal_response_callback)
        r2_gripper_future = self.robot2_gripper_control_client.send_goal_async(gripper_msg)
        r2_gripper_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass

        self.robot1_nav_to_pose_client.wait_for_server()
        self.robot2_nav_to_pose_client.wait_for_server()

        r1_nav_to_pose_msg = NavToPose.Goal()
        r2_nav_to_pose_msg = NavToPose.Goal()

        q1_goal, q2_goal = self.get_q_goals(approach_object_msg.object_pose, object_radius)

        r1_nav_to_pose_msg.pose.x = q1_goal[0]
        r1_nav_to_pose_msg.pose.y = q1_goal[1]
        r1_nav_to_pose_msg.pose.theta = q1_goal[2]

        r2_nav_to_pose_msg.pose.x = q2_goal[0]
        r2_nav_to_pose_msg.pose.y = q2_goal[1]
        r2_nav_to_pose_msg.pose.theta = q2_goal[2]

        r1_nav_to_pose_future = self.robot1_nav_to_pose_client.send_goal_async(r1_nav_to_pose_msg)
        r1_nav_to_pose_future.add_done_callback(self.goal_response_callback)
        r2_nav_to_pose_future = self.robot2_nav_to_pose_client.send_goal_async(r2_nav_to_pose_msg)
        r2_nav_to_pose_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass

        time.sleep(2)

        self.robot1_gripper_control_client.wait_for_server()
        self.robot2_gripper_control_client.wait_for_server()

        gripper_msg = ControlGripper.Goal()
        gripper_msg.open = False

        r1_gripper_future = self.robot1_gripper_control_client.send_goal_async(gripper_msg)
        r1_gripper_future.add_done_callback(self.goal_response_callback)
        r2_gripper_future = self.robot2_gripper_control_client.send_goal_async(gripper_msg)
        r2_gripper_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass

    def get_q_goals(self, object_pose, object_radius):

        distance_to_object = object_radius + 0.15

        q1_x = object_pose.x + distance_to_object*np.sin(object_pose.theta)
        q1_y = object_pose.y - distance_to_object*np.cos(object_pose.theta)
        q1_theta = np.pi/2 + object_pose.theta

        q2_x = object_pose.x - distance_to_object*np.sin(object_pose.theta)
        q2_y = object_pose.y + distance_to_object*np.cos(object_pose.theta)
        q2_theta = object_pose.theta - np.pi/2 

        return (q1_x, q1_y, q1_theta), (q2_x, q2_y, q2_theta)



    def goal_response_callback(self, future):

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        _get_result_future = goal_handle.get_result_async()
        _get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        self.executing = False

        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))

        

def main(args=None):

    rclpy.init(args=args)

    caging_client = CagingClient()

    thread = threading.Thread(target=rclpy.spin, args=(caging_client,))
    thread.start()

    caging_client.caging()

    #rclpy.spin(caging_client, executor=MultiThreadedExecutor())

    caging_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
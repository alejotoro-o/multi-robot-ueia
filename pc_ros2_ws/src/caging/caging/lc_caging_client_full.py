import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState, ChangeState
from interfaces.msg import Pose2D
from interfaces.action import NavToPose, ApproachObject, ControlGripperAngle, FollowTrajectory

import threading
import numpy as np
import time

class CagingClient(Node):

    def __init__(self):

        super().__init__('caging_client')

        self.declare_parameter('object_pose', [0.0,0.0,0.0])
        self.declare_parameter('object_radius', 0.0)
        self.declare_parameter('gripper_angle', 60)
        self.gripper_angle = self.get_parameter('gripper_angle').get_parameter_value().integer_value
        self.declare_parameter('num_waypoints', 20)
        self.num_waypoints = self.get_parameter('num_waypoints').get_parameter_value().integer_value
        self.declare_parameter('trajectory_time', 10.0)
        self.declare_parameter('trajectory_orientation', 3*np.pi/4)
        self.trajectory_orientation = self.get_parameter('trajectory_orientation').get_parameter_value().double_value

        self.approach_object_client = ActionClient(self, ApproachObject, "approach_object")

        self.robot1_gripper_control_client = ActionClient(self, ControlGripperAngle, "robot1/control_gripper")
        self.robot2_gripper_control_client = ActionClient(self, ControlGripperAngle, "robot2/control_gripper")

        self.robot1_nav_to_pose_client = ActionClient(self, NavToPose, "robot1/nav_to_pose")
        self.robot2_nav_to_pose_client = ActionClient(self, NavToPose, "robot2/nav_to_pose")

        self.caging_follow_trajectory_client = ActionClient(self, FollowTrajectory, "follow_trajectory")

        ## LifeCycle Clients
        self.robot1_lqr_get_state_client = self.create_client(GetState, "robot1/lc_lqr_controller/get_state")
        self.robot1_lqr_change_state_client = self.create_client(ChangeState, "robot1/lc_lqr_controller/change_state")
        self.robot2_lqr_get_state_client = self.create_client(GetState, "robot2/lc_lqr_controller/get_state")
        self.robot2_lqr_change_state_client = self.create_client(ChangeState, "robot2/lc_lqr_controller/change_state")

        self.robot2_caging_controller_get_state_client = self.create_client(GetState, "lc_caging_controller/get_state")
        self.robot2_caging_controller_change_state_client = self.create_client(ChangeState, "lc_caging_controller/change_state")

        self.caging_lqr_get_state_client = self.create_client(GetState, "caging_lqr_controller/get_state")
        self.caging_lqr_change_state_client = self.create_client(ChangeState, "caging_lqr_controller/change_state")

        self.executing = False

    def caging(self):

        self.activate_lqr()

        time.sleep(10)

        self.grab_object()

        self.deactivate_lqr()

        time.sleep(2)

        self.activate_caging_controller()

        time.sleep(2)

        self.activate_caging_lqr()

        ## Follow trajectory after caging object
        path = self.generate_caging_trajectory()

        goal_msg = FollowTrajectory.Goal()
        goal_msg.path = [path[0]]
        goal_msg.time = 10.0

        self.caging_follow_trajectory_client.wait_for_server()
        follow_trajectory_future = self.caging_follow_trajectory_client.send_goal_async(goal_msg)
        follow_trajectory_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass

        goal_msg.path = path
        goal_msg.time = self.get_parameter('trajectory_time').get_parameter_value().double_value

        self.caging_follow_trajectory_client.wait_for_server()
        follow_trajectory_future = self.caging_follow_trajectory_client.send_goal_async(goal_msg)
        follow_trajectory_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass


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

        gripper_msg = ControlGripperAngle.Goal()
        gripper_msg.angle = 0

        r1_gripper_future = self.robot1_gripper_control_client.send_goal_async(gripper_msg)
        r1_gripper_future.add_done_callback(self.goal_response_callback)
        r2_gripper_future = self.robot2_gripper_control_client.send_goal_async(gripper_msg)
        r2_gripper_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass

        time.sleep(2)

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

        time.sleep(4)

        self.robot1_gripper_control_client.wait_for_server()
        self.robot2_gripper_control_client.wait_for_server()

        gripper_msg = ControlGripperAngle.Goal()
        gripper_msg.angle = self.gripper_angle

        r1_gripper_future = self.robot1_gripper_control_client.send_goal_async(gripper_msg)
        r1_gripper_future.add_done_callback(self.goal_response_callback)
        r2_gripper_future = self.robot2_gripper_control_client.send_goal_async(gripper_msg)
        r2_gripper_future.add_done_callback(self.goal_response_callback)

        self.executing = True

        while self.executing:
            pass

        time.sleep(2)

    def get_q_goals(self, object_pose, object_radius):

        distance_to_object = object_radius + 0.15

        q1_x = object_pose.x + distance_to_object*np.sin(object_pose.theta)
        q1_y = object_pose.y - distance_to_object*np.cos(object_pose.theta)
        q1_theta = np.pi/2 + object_pose.theta

        q2_x = object_pose.x - distance_to_object*np.sin(object_pose.theta)
        q2_y = object_pose.y + distance_to_object*np.cos(object_pose.theta)
        q2_theta = object_pose.theta - np.pi/2 

        return (q1_x, q1_y, q1_theta), (q2_x, q2_y, q2_theta)

    def generate_caging_trajectory(self):

        points = np.linspace(0, 2*np.pi, self.num_waypoints)

        waypoints = np.array([1*np.cos(points), 1*np.sin(points)])
        waypoints = list(waypoints.T)

        path = []

        for w in waypoints:

            waypoint = Pose2D()
            waypoint.x = float(w[0])
            waypoint.y = float(w[1])
            if self.trajectory_orientation == -1:
                waypoint.theta = np.arctan2(w[1],w[0]) + np.pi/2
            else:
                waypoint.theta = self.trajectory_orientation

            path.append(waypoint)

        return path

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

    ############################
    ## Manage LifeCycle Nodes ##
    ############################
    
    ## LQR Controllers
    def get_lqr_state(self):

        ## Get State
        get_state_request = GetState.Request()

        r1_get_state_future = self.robot1_lqr_get_state_client.call_async(get_state_request)
        r2_get_state_future = self.robot2_lqr_get_state_client.call_async(get_state_request)

        while not (r1_get_state_future.done() and r2_get_state_future.done()):
            pass

        r1_state = r1_get_state_future.result().current_state.label
        r2_state = r2_get_state_future.result().current_state.label
        self.get_logger().info("Robot 1 LQR Controller state: %s" % r1_state)
        self.get_logger().info("Robot 2 LQR Controller state: %s" % r2_state)
        
    def change_lqr_state(self, transition):

        ## Change State
        change_state_request = ChangeState.Request()
        change_state_request.transition.id = transition

        r1_change_state_future = self.robot1_lqr_change_state_client.call_async(change_state_request)
        r2_change_state_future = self.robot2_lqr_change_state_client.call_async(change_state_request)

        while not (r1_change_state_future.done() and r2_change_state_future.done()):
            pass

        r1_transition_result = r1_change_state_future.result().success
        r2_transition_result = r2_change_state_future.result().success
        self.get_logger().info("Robot 1 LQR Controller transition result: %s" % r1_transition_result)
        self.get_logger().info("Robot 2 LQR Controller transition result: %s" % r2_transition_result)

    ## Caging Controller
    def get_caging_controller_state(self):
        
        ## Get State
        get_state_request = GetState.Request()

        r2_get_state_future = self.robot2_caging_controller_get_state_client.call_async(get_state_request)

        while not r2_get_state_future.done():
            pass

        r2_state = r2_get_state_future.result().current_state.label
        self.get_logger().info("Robot 1 Caging Controller Controller state: %s" % r2_state)


    def change_caging_controller_state(self, transition):
        
        ## Change State
        change_state_request = ChangeState.Request()
        change_state_request.transition.id = transition

        r2_change_state_future = self.robot2_caging_controller_change_state_client.call_async(change_state_request)

        while not r2_change_state_future.done():
            pass

        r2_transition_result = r2_change_state_future.result().success
        self.get_logger().info("Robot 2 Caging Controller transition result: %s" % r2_transition_result)

    ## Caging LQR Controller
    def get_caging_lqr_state(self):
        
        ## Get State
        get_state_request = GetState.Request()

        r2_get_state_future = self.caging_lqr_get_state_client.call_async(get_state_request)

        while not r2_get_state_future.done():
            pass

        r2_state = r2_get_state_future.result().current_state.label
        self.get_logger().info("Caging LQR Controller state: %s" % r2_state)


    def change_caging_lqr_state(self, transition):
        
        ## Change State
        change_state_request = ChangeState.Request()
        change_state_request.transition.id = transition

        r2_change_state_future = self.caging_lqr_change_state_client.call_async(change_state_request)

        while not r2_change_state_future.done():
            pass

        r2_transition_result = r2_change_state_future.result().success
        self.get_logger().info("Caging LQR Controller transition result: %s" % r2_transition_result)

        
    def activate_lqr(self):

        while not (self.robot1_lqr_get_state_client.wait_for_service() and self.robot2_lqr_get_state_client.wait_for_service() and self.robot1_lqr_change_state_client.wait_for_service() and self.robot2_lqr_change_state_client.wait_for_service()):
            self.get_logger().info("Waiting for LQR LifeCycle Nodes services...")

        ## Change state
        transition = Transition.TRANSITION_CONFIGURE
        self.change_lqr_state(transition)
        self.get_lqr_state()

        transition = Transition.TRANSITION_ACTIVATE
        self.change_lqr_state(transition)
        self.get_lqr_state()

    def deactivate_lqr(self):

        while not (self.robot1_lqr_get_state_client.wait_for_service() and self.robot2_lqr_get_state_client.wait_for_service()):
            self.get_logger().info("Waiting for LQR LifeCycle Nodes services...")

        ## Change state
        transition = Transition.TRANSITION_DEACTIVATE
        self.change_lqr_state(transition)
        self.get_lqr_state()

        transition = Transition.TRANSITION_CLEANUP
        self.change_lqr_state(transition)
        self.get_lqr_state()

    def activate_caging_controller(self):
        
        while not self.robot2_caging_controller_get_state_client.wait_for_service():
            self.get_logger().info("Waiting for Caging Controller LifeCycle Node services...")

        ## Change state
        transition = Transition.TRANSITION_CONFIGURE
        self.change_caging_controller_state(transition)
        self.get_caging_controller_state()

        transition = Transition.TRANSITION_ACTIVATE
        self.change_caging_controller_state(transition)
        self.get_caging_controller_state()

    def activate_caging_lqr(self):
        
        while not self.caging_lqr_get_state_client.wait_for_service():
            self.get_logger().info("Waiting for Caging Controller LifeCycle Node services...")

        ## Change state
        transition = Transition.TRANSITION_CONFIGURE
        self.change_caging_lqr_state(transition)
        self.get_caging_lqr_state()

        transition = Transition.TRANSITION_ACTIVATE
        self.change_caging_lqr_state(transition)
        self.get_caging_lqr_state()

        

def main(args=None):

    rclpy.init(args=args)

    caging_client = CagingClient()

    thread = threading.Thread(target=rclpy.spin, args=(caging_client,))
    thread.start()

    caging_client.caging()

    caging_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist, Pose, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Int8

from tf2_ros import TransformBroadcaster

import time
from controller import Supervisor
import numpy as np
from scipy.spatial.transform import Rotation as R

L_X = 0.0845
L_Y = 0.08
WHEEL_RADIUS = 0.0325

class MultiRobotDriverGripper:
    def init(self, webots_node, properties):

        ## Robot node
        self.__robot = webots_node.robot  

        self.robot_name = self.__robot.getName()

        ## Supervisor node
        self.supervisor = Supervisor()
        self.robot_node = self.supervisor.getFromDef(self.robot_name)
        self.trans_field = self.robot_node.getField("translation")
        self.rot_field = self.robot_node.getField("rotation")     

        ## Configure position sensors (Encoders)
        self.__encoder1 = self.__robot.getDevice('encoder1')
        self.__encoder1.enable(32)

        self.__encoder2 = self.__robot.getDevice('encoder2')
        self.__encoder2.enable(32)

        self.__encoder3 = self.__robot.getDevice('encoder3')
        self.__encoder3.enable(32)

        self.__encoder4 = self.__robot.getDevice('encoder4')
        self.__encoder4.enable(32)

        ## Gripper encoders and motors

        self.__r_gripper_motor = self.__robot.getDevice('r_grip_motor')
        self.__r_gripper_motor.setPosition(float('inf'))
        self.__r_gripper_motor.setVelocity(0)
        self.__l_gripper_motor = self.__robot.getDevice('l_grip_motor')
        self.__l_gripper_motor.setPosition(float('inf'))
        self.__l_gripper_motor.setVelocity(0)

        self.gripper_status = False
        self.moving_gripper = False

        ## Configure motors
        self.__front_left_motor = self.__robot.getDevice('motor1')
        self.__front_right_motor = self.__robot.getDevice('motor2')
        self.__back_left_motor = self.__robot.getDevice('motor3')
        self.__back_right_motor = self.__robot.getDevice('motor4')

        self.__front_left_motor.setPosition(float('inf'))
        self.__front_left_motor.setVelocity(0)

        self.__front_right_motor.setPosition(float('inf'))
        self.__front_right_motor.setVelocity(0)

        self.__back_left_motor.setPosition(float('inf'))
        self.__back_left_motor.setVelocity(0)

        self.__back_right_motor.setPosition(float('inf'))
        self.__back_right_motor.setVelocity(0)

        self.__target_twist = Twist()

        ## Node configuration
        rclpy.init(args=None)
        self.__node = rclpy.create_node(self.robot_name + '_driver')

        self.__node.create_subscription(Twist, self.robot_name + '/cmd_vel', self.__cmd_vel_callback, 1)
        self.__node.create_subscription(Int8, self.robot_name + '/move_gripper', self.__move_gripper_callback, 1)

        self.__publisher = self.__node.create_publisher(JointState, self.robot_name + '/wheels_encoders', 1)
        self.__vel_publisher = self.__node.create_publisher(Twist, self.robot_name + '/vel_raw', 1)
        self.__pose_publisher = self.__node.create_publisher(Pose, self.robot_name + '/pose', 1)

        self.tf_broadcaster = TransformBroadcaster(self.__node)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def __move_gripper_callback(self, cmd):

        self.gripper_status = cmd.data

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        ## Set motor speeds
        V_x = self.__target_twist.linear.x
        V_y = self.__target_twist.linear.y
        thetap = self.__target_twist.angular.z

        command_motor_front_left = (V_x - V_y - (L_X + L_Y)*thetap) / WHEEL_RADIUS
        command_motor_front_right = (V_x + V_y + (L_X + L_Y)*thetap) / WHEEL_RADIUS
        command_motor_back_left = (V_x + V_y - (L_X + L_Y)*thetap) / WHEEL_RADIUS
        command_motor_back_right = (V_x - V_y + (L_X + L_Y)*thetap) / WHEEL_RADIUS

        self.__front_left_motor.setVelocity(command_motor_front_left) 
        self.__front_right_motor.setVelocity(command_motor_front_right) 
        self.__back_left_motor.setVelocity(command_motor_back_left)
        self.__back_right_motor.setVelocity(command_motor_back_right) 

        ## Publish joint states
        joint_states_header = Header()
        joint_states_header.stamp = self.__node.get_clock().now().to_msg()
        joint_states_header.frame_id = ''

        joint_states = JointState()
        joint_states.header = joint_states_header
        joint_states.name = ['motor2', 'motor1', 'motor4', 'motor3']
        joint_states.position = [
            float(self.__encoder2.getValue()),
            float(self.__encoder1.getValue()),
            float(self.__encoder4.getValue()),
            float(self.__encoder3.getValue())
        ]

        self.__publisher.publish(joint_states)
        self.__vel_publisher.publish(self.__target_twist)

        ## Publish pose
        pose_msg = Pose()

        trans = self.trans_field.getSFVec3f()

        rot = self.rot_field.getSFRotation()
        r = R.from_rotvec(np.sign(rot[2])*rot[3]*np.array([0,0,1]))

        # pose_msg.header.frame_id = 'map'
        # pose_msg.header.stamp = self.__node.get_clock().now().to_msg()

        pose_msg.position.x = trans[0]
        pose_msg.position.y = trans[1]
        pose_msg.position.z = trans[2]

        pose_msg.orientation.w = r.as_quat()[3]
        pose_msg.orientation.x = r.as_quat()[0]
        pose_msg.orientation.y = r.as_quat()[1]
        pose_msg.orientation.z = r.as_quat()[2]

        self.__pose_publisher.publish(pose_msg)

        ## TF transform
        t = TransformStamped()

        t.header.stamp = self.__node.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]

        t.transform.rotation.x = r.as_quat()[0]
        t.transform.rotation.y = r.as_quat()[1]
        t.transform.rotation.z = r.as_quat()[2]
        t.transform.rotation.w = r.as_quat()[3]

        self.tf_broadcaster.sendTransform(t)
        
        if self.gripper_status == 1:
            self.__r_gripper_motor.setVelocity(-1)
            self.__l_gripper_motor.setVelocity(1)  
        elif self.gripper_status == -1:
            self.__r_gripper_motor.setVelocity(1)
            self.__l_gripper_motor.setVelocity(-1)  
        elif self.gripper_status == 0:
            self.__r_gripper_motor.setVelocity(0)
            self.__l_gripper_motor.setVelocity(0)  

    
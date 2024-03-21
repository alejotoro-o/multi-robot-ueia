#!/usr/bin/env python
# encoding: utf-8

#public lib
from Rosmaster_Lib import Rosmaster

#ros lib
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from rclpy.clock import Clock

from interfaces.action import ControlGripperAngle

class yahboomcar_driver(Node):

    def __init__(self):

        super().__init__('robot_driver')
        self.car = Rosmaster()
        self.car.set_car_type(1)

        #create subcriber
        self.sub_cmd_vel = self.create_subscription(Twist,"cmd_vel",self.cmd_vel_callback,1)

        self.action_server = ActionServer(self, ControlGripperAngle, 'control_gripper', self.control_gripper_callback)

        self.car.create_receive_threading()
        self.car.set_pwm_servo(1, 0)

    def cmd_vel_callback(self,msg):

        vx = msg.linear.x*1.0
        vy = msg.linear.y*1.0
        angular = msg.angular.z*1.0 

        self.car.set_car_motion(vx, vy, angular)

    def control_gripper_callback(self, goal_handle):

        self.car.set_pwm_servo(1, goal_handle.request.angle)

        goal_handle.succeed()

        goal_result = ControlGripperAngle.Result()

        goal_result.result = "Ok"

        return goal_result
        
                    
def main():
    rclpy.init() 
    driver = yahboomcar_driver()
    rclpy.spin(driver)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
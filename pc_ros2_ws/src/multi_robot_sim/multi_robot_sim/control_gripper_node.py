import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from std_msgs.msg import Int8

from interfaces.action import ControlGripper

import time

class ControlGripperServer(Node):

    def __init__(self):

        super().__init__('control_gripper_server')

        self.gripper_comand_publisher = self.create_publisher(Int8, 'move_gripper', 1)

        self.action_server = ActionServer(self, ControlGripper, 'control_gripper', self.control_gripper_callback)
        self.gripper_status = False

    def control_gripper_callback(self, goal_handle):

        cmd = Int8()

        if self.gripper_status != goal_handle.request.open:
            if goal_handle.request.open: 
                cmd.data = 1
                self.gripper_comand_publisher.publish(cmd) 
                self.gripper_status = True
            else:
                cmd.data = -1
                self.gripper_comand_publisher.publish(cmd)
                self.gripper_status = False

        time.sleep(1.5)

        cmd.data = 0
        self.gripper_comand_publisher.publish(cmd)

        goal_handle.succeed()

        goal_result = ControlGripper.Result()

        goal_result.result = "Ok"

        return goal_result
    
def main(args=None):

    rclpy.init(args=args)

    control_gripper_server = ControlGripperServer()

    rclpy.spin(control_gripper_server)

    control_gripper_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
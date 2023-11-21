import math
import rclpy
import numpy as np
from matplotlib import pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import kinpy as kp



class UR(Node):

    def __init__(self):
        super().__init__('ur_controller')

        self.joint_states = None
        self.chain = None

        # Subscribers
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.cb_joint_states,
            10)

        self.sub_joint_states = self.create_subscription(
            String,
            '/robot_description_latch',
            self.cb_robot_description,
            10)

        # Publishers
        self.pub_joint_states = self.create_publisher(
            JointState,
            '/set_joint_states',
            10)



    # Callback for pose
    def cb_joint_states(self, msg):
        self.joint_states = msg
        #print(self.joint_states)

    # Callback for robot description
    def cb_robot_description(self, msg):
        if self.chain is None:
            self.chain = kp.build_serial_chain_from_urdf(msg.data, 'tool0')
            print(self.chain.get_joint_parameter_names())
            print(self.chain)



    # Move robot to configuration
    def to_configuration(self, q):
        # Wait for position to be received
        loop_rate = self.create_rate(100.0, self.get_clock()) # Hz
        while self.joint_states is None and rclpy.ok():
            self.get_logger().info('Waiting for joint states...')
            rclpy.spin_once(self)

        msg = self.joint_states
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = q

        self.pub_joint_states.publish(msg)

    def calc_kinematics(self):
        # Wait for position to be received
        loop_rate = self.create_rate(100.0, self.get_clock()) # Hz
        while self.joint_states is None and rclpy.ok():
            self.get_logger().info('Waiting for joint states...')
            rclpy.spin_once(self)
        while self.chain is None and rclpy.ok():
            self.get_logger().info('Waiting for chain...')
            rclpy.spin_once(self)

        # Configuraion
        q = self.joint_states.position

        # TCP pose
        p = self.chain.forward_kinematics(q)
        return p


def main(args=None):
    rclpy.init(args=args)
    arm = UR()

    arm.to_configuration([-1.28, 4.41, 1.54, -1.16, -1.56, 0.0])
    print(arm.calc_kinematics())
    rclpy.spin(arm)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

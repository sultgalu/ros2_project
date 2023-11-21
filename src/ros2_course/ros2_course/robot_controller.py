import math
import rclpy
import numpy as np
from matplotlib import pyplot as plt
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Robot(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.cmd_vel = None
        self.laser_scan = None
        # Subscribers
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/model/vehicle_blue/cmd_vel',
            self.cb_cmd_vel,
            10)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/model/vehicle_blue/cmd_vel',
            10)

        self.sub_laser_scan = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.cb_laser_scan,
            10)

    # Callback for pose
    def cb_laser_scan(self, msg):
        self.laser_scan = msg
        # self.get_logger().info('got laser info')
        #print(self.joint_states)

    # Callback for pose
    def cb_cmd_vel(self, msg):
        self.cmd_vel = msg
        #print(self.joint_states)

    # Move robot to configuration
    def to_configuration(self, q):
        # Wait for position to be received
        loop_rate = self.create_rate(100.0, self.get_clock()) # Hz
        while self.cmd_vel is None and rclpy.ok():
            self.get_logger().info('Waiting for cmd_vel...')
            rclpy.spin_once(self)

        msg = self.cmd_vel
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = q

        self.pub_cmd_vel.publish(msg)

    def wait_for_crash(self):
        while self.laser_scan is None and rclpy.ok():
            self.get_logger().info('Waiting for laser_scan...')
            rclpy.spin_once(self)


        avoiding = False
        shouldStop = False
        while not shouldStop:
            ds = [None] * len(self.laser_scan.ranges)
            ws = [None] * len(self.laser_scan.ranges)
            minD = float("inf")
            minIdx = -1
            minW = float("inf")
            for idx, range in enumerate(self.laser_scan.ranges):
                angle = self.laser_scan.angle_min + self.laser_scan.angle_increment * idx
                d = math.cos(angle) * range
                w = math.sin(angle) * range
                ds[idx] = d if abs(w) < 0.9 else float("inf")
                ws[idx] = abs(w)
                if (ds[idx] < minD):
                    minD = ds[idx]
                    minIdx = idx
                    minW = w
                # if (abs(w) < 0.9):
                    # shouldStop = True
                    # robot.pub_cmd_vel.publish(Twist())
                    # break
                    # self.get_logger().info('{0:.2f} {1:.2f} {2:.2f} CRASH'.format(min(ds),min(ws), min(self.laser_scan.ranges)))
            if (min(ds) < 2.0):
                self.get_logger().info('{0:.2f} {1:.2f} {2:.2f} CRASH'.format(min(ds), min(ws), min(self.laser_scan.ranges)))
                avoiding = True
                twist = Twist()
                twist.angular.z = 1.0 if (minW < 0.0) else -1.0
                # twist.linear.x = 1.0
                self.pub_cmd_vel.publish(twist)
            elif avoiding:
                avoiding = False
                twist = Twist()
                twist.linear.x = 1.0
                self.pub_cmd_vel.publish(twist)
            else:
                self.get_logger().info('{0:.2f} {1:.2f} {2:.2f}'.format(min(ds), min(ws), min(self.laser_scan.ranges)))
            rclpy.spin_once(self)

        self.get_logger().info('CRASH INCOMIN')

def main(args=None):
    rclpy.init(args=args)
    robot = Robot()

    robot.wait_for_crash()

    #twist = Twist()
    #twist.linear.x = -0.1
    #robot.pub_cmd_vel.publish(twist)
    # arm.to_configuration([-1.28, 4.41, 1.54, -1.16, -1.56, 0.0])
    # print(arm.calc_kinematics())
    rclpy.spin(robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

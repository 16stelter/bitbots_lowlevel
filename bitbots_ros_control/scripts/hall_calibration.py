#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from bitbots_msgs.msg import FloatStamped, JointCommand
import yaml
import ament_index_python


class Calibration(Node):
    def __init__(self):
        rclpy.init(args=None)
        super().__init__('hall_calibration')
        self.cfgpath = ament_index_python.get_package_share_directory("bitbots_ros_control") + "/config/hall.yaml"

        self.lhall = 0
        self.rhall = 0

        self.loffset = 0
        self.roffset = 0

        self.lsub = self.create_subscription(FloatStamped, "/hall/left/raw", self.l_cb, 1)
        self.rsub = self.create_subscription(FloatStamped, "/hall/right/raw", self.r_cb, 1)
        self.mgpub = self.create_publisher(JointCommand, "DynamixelController/command", 1)

        self.main()

    def l_cb(self, msg):
        self.lhall = 1088 - msg.value  # Dynamixel 2048 = 0 degrees, this is PWM offset
        return

    def r_cb(self, msg):
        self.rhall = 1088 - msg.value  # Dynamixel 2048 = 0 degrees, this is PWM offset
        return

    def main(self):
        self.get_logger().info("Hall Sensor Calibration Tool \n \n")
        self.get_logger().info("Please make sure the robot is not touching the ground, "
                                    "then press any key to continue.\n")
        input()
        self.get_logger().info("Config path is " + self.cfgpath)
        self.get_logger().info("Setting all motors to zero.")
        # Set all motors to zero, first calibration point
        msg = JointCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ["RShoulderPitch", "LShoulderPitch", "RShoulderRoll", "LShoulderRoll", "RElbow",
                           "LElbow", "RHipYaw", "LHipYaw", "RHipRoll", "LHipRoll", "RHipPitch",
                           "LHipPitch", "RKnee", "LKnee", "RAnklePitch", "LAnklePitch", "RAnkleRoll",
                           "LAnkleRoll", "HeadPan", "HeadTilt"]
        msg.positions = [0.0] * 20
        msg.velocities = [5.0] * 20
        msg.accelerations = [-1.0] * 20
        msg.max_currents = [-1.0] * 20
        self.mgpub.publish(msg)
        self.get_logger().info("Waiting 3 seconds...")
        time.sleep(3)
        for i in range (100):
            rclpy.spin_once(self)
        loffset = self.lhall
        roffset = self.rhall
        self.get_logger().info("Left offset is " + str(loffset))
        self.get_logger().info("Right offset is " + str(roffset))
        self.get_logger().info("Gathering gradient...")
        # second calibration point to get gradient
        msg2 = JointCommand()
        msg2.header.stamp = self.get_clock().now().to_msg()
        msg2.joint_names = ["LKnee", "RKnee"]
        msg2.positions = [1.0, -1.0]
        msg2.velocities = [-1.0, -1.0]
        msg2.accelerations = [-1.0, -1.0]
        msg2.max_currents = [-1.0, -1.0]
        self.mgpub.publish(msg2)
        self.get_logger().info("Waiting 3 seconds...")
        time.sleep(3)
        for i in range (100):
            rclpy.spin_once(self)
        lgrad = 1.0 / (self.lhall - loffset)
        rgrad = -1.0 / (self.rhall - roffset)  # Im not sure why, but this is flipped...
        self.get_logger().info("Left gradient is " + str(lgrad))
        self.get_logger().info("Right gradient is " + str(rgrad))
        self.get_logger().info("Going back to init position...")
        # go back to init position
        msg.header.stamp = self.get_clock().now().to_msg()
        self.mgpub.publish(msg)
        self.get_logger().info("Writing to config file...")
        output = {
            'hall_converter': {
                'ros__parameters': {
                    'left_topic': '/hall/left',
                    'right_topic': '/hall/right',
                    'l_offset': loffset,
                    'r_offset': roffset,
                    'l_gradient': lgrad,
                    'r_gradient': rgrad
                }
            }
        }
        with open(self.cfgpath,'w') as f:
            yaml.dump(output, f)

        self.get_logger().info("Sensors calibrated successfully!")


if __name__ == "__main__":
    node = Calibration()
    exec = rclpy.executors.MultiThreadedExecutor()
    exec.add_node(node)
    exec.spin()







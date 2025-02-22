#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math

from bitbots_msgs.msg import JointCommand


DYNAMIXEL_CMD_TOPIC = "/DynamixelController/command"
JOINT_NAME = "LAnkleRoll"
PUBLISH_RATE = 1000

# sin function
FREQUENCY = 0.5
AMPLITUDE = 72 #degree

if __name__ == "__main__":
    msg = JointCommand(
        joint_names=[JOINT_NAME],
        velocities=[-1],
        accelerations=[-1],
        max_currents=[-1])

    rclpy.init(args=None)
    pub = self.create_publisher(JointCommand, DYNAMIXEL_CMD_TOPIC, 1)

    rate = self.create_rate(PUBLISH_RATE)
    while rclpy.ok():
        time = self.get_clock().now()
        position = math.radians(AMPLITUDE) * math.sin(2 * math.pi * FREQUENCY * time.to_sec())

        msg.header.stamp = time
        msg.positions=[position]
        pub.publish(msg)
        rate.sleep()
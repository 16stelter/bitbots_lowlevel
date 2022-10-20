#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from bitbots_msgs.msg import FloatStamped, JointCommand
import math
import pandas as pd

'''
Makes a servo move in a sine wave.
'''
class SineWave(Node):

    def __init__(self):
        rclpy.init(args=None)
        super().__init__('sinewave')
        self.mgpub = self.create_publisher(JointCommand, "DynamixelController/command", 1)

        self.main()

    def main(self):
        i = 0
        msg = JointCommand()
        msg.joint_names = ["LKnee"]
        msg.velocities = [5.0]
        msg.accelerations = [-1.0]
        msg.max_currents = [-1.0]
        while True:
            msg.positions = [(math.sin(i) + 1.0)]
            self.mgpub.publish(msg)
            i += 0.01
            time.sleep(0.01)



if __name__ == "__main__":
    node = SineWave()
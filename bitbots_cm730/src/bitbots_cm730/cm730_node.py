#!/usr/bin/env python
# -*- coding:utf-8 -*-
import time

import math
# from Cython.Includes.cpython.exc import PyErr_CheckSignals

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Temperature, Imu
from humanoid_league_msgs.msg import AdditionalServoData
from humanoid_league_msgs.msg import Speak

from bitbots_cm730.cm730 import CM730
from bitbots_cm730.srv import SwitchMotorPower, SetLEDs
from bitbots_speaker.speaker import speak
from bitbots_common.pose.pypose import PyPose as Pose
from bitbots_common.utilCython.pydatavector import PyIntDataVector as IntDataVector
from bitbots_common.utilCython.pydatavector import PyDataVector as DataVector
from bitbots_buttons.msg import Buttons

import rospy


class CM730Node:
    """
    Takes core of the communication with the CM730 (and therefore with the servos). Constantly updates servo data
    and sets goal values to the servos.
    """

    def __init__(self):
        rospy.init_node('bitbots_cm730', anonymous=False)

        # --- Class Variables ---

        self.goal_pose = Pose()
        self.cm_730 = CM730()
        self.led_eye = (0, 0, 0)
        self.led_head = (0, 0, 0)

        # --- Initialize Topics ---
        rospy.Subscriber("/motion_motor_goals", JointTrajectory, self.update_motor_goals)
        self.joint_publisher = rospy.Publisher('/current_motor_positions', JointState, queue_size=10)
        self.speak_publisher = rospy.Publisher('/speak', Speak, queue_size=10)
        self.temp_publisher = rospy.Publisher('/servo_data', AdditionalServoData, queue_size=10)
        self.imu_publisher = rospy.Publisher('/imu', Imu, queue_size=10)
        self.button_publisher = rospy.Publisher('/buttons', Buttons, queue_size=10)
        self.motor_power_service = rospy.Service("switch_motor_power", SwitchMotorPower,
                                                 self.switch_motor_power_service_call)
        self.led_service = rospy.Service("set_leds", SetLEDs,
                                         self.set_LED_service_call)

        speak("CM730 connected", self.speak_publisher)

        # --- Setting Params ---
        joints = rospy.get_param("/joints")
        self.joint_limits = {}
        # problem is, that the number of motors is not known at build time, so write them into params now
        for motor in joints:
            min_value = -180
            max_value = 180
            if 'max' in motor['limits']:
                max_value = motor['limits']['max']
            if 'min' in motor['limits']:
                min_value = motor['limits']['min']
            rospy.set_param("/joints/" + str(motor['name']), {'min': min_value, 'max': max_value})
            self.joint_limits[motor['name']] = {'min': min_value, 'max': max_value}
        # start the endless loop
        self.update_forever()

    def update_motor_goals(self, msg):
        """ Callback for subscription on motorgoals topic.
        We can only handle the first point of a JointTrajectory :( """
        motor_goals = []
        joints = msg.joint_names
        # we can handle only one position, no real trajectory
        positions = msg.points[0].positions
        i = 0
        for joint in joints:
            if positions[i] > self.joint_limits[joint]['max']:
                motor_goals.append(self.joint_limits[joint]['max'])
                rospy.logdebug("Joint command over max. Joint: %s Position: %f", (joint, positions[i]))
            elif positions[i] < self.joint_limits[joint]['min']:
                motor_goals.append(self.joint_limits[joint]['min'])
                rospy.logdebug("Joint command under min. Joint: %s Position: %f", (joint, positions[i]))
            else:
                motor_goals.append(math.degrees(positions[i]))
            i += 1
        # update goal pose accordingly
        self.goal_pose.set_goals(joints, motor_goals)

    def update_forever(self):
        """ Calls :func:`update_once` in an infinite loop """
        iteration = 0
        duration_avg = 0
        start = time.time()

        while not rospy.is_shutdown():
            self.update_once()

            # Count to get the update frequency
            iteration += 1
            if iteration < 100:
                continue

            if duration_avg > 0:
                duration_avg = 0.5 * duration_avg + 0.5 * (time.time() - start)
            else:
                duration_avg = (time.time() - start)

            rospy.loginfo("Updates/Sec %f", iteration / duration_avg)
            iteration = 0
            start = time.time()

    def update_once(self):
        """ Updates sensor data with :func:`update_sensor_data`, publishes the data and sends the motor commands.

            The sensordata from the last iteration will be provided as smoothed values in
            :attr:`smooth_accel` and :attr:`smooth_gyro`.
        """
        # get sensor data
        robo_pose, gyro, accel, button1, button2 = self.update_sensor_data()

        # Send Messages to ROS
        if robo_pose is not None:
            self.publish_joints(robo_pose)
        # todo self.publish_additional_servo_data()
        if gyro is not None:
            self.publish_imu(gyro, accel)
        if button1 is not None:
            # we have to check this because we don't update the buttons everytime
            self.publish_buttons(button1, button2)

        # send new position to servos
        self.cm_730.apply_goal_pose(self.goal_pose)

    def update_sensor_data(self):
        robo_pose = Pose()
        raw_accel = None
        raw_gyro = None
        # first get data
        result, cid_all_values = self.cm_730.sensor_data_read()

        if not result:
            return None, None, None, None, None
        if result == -1:
            # this means the motion is stuck
            rospy.logerr("IO Error. Reading error on CM730. Motion stuck!")
            speak("IO Error. Reading error on CM730. Motion stuck!", self.speak_publisher)
            exit("IO Error. Reading error on CM730. Motion stuck!")

        # parse data
        button, gyro, accel, robo_pose = self.cm_730.parse_sensor_data(result, cid_all_values)

        if button == -1:
            # low voltage error
            speak("Warning: Low Voltage! System Exit!")
            rospy.logerr("SYSTEM EXIT: LOW VOLTAGE")
            raise SystemExit("SYSTEM EXIT: LOW VOLTAGE " + str(gyro / 10))

        if accel is not None:
            raw_accel = accel - IntDataVector(512, 512, 512)
        if gyro is not None:
            raw_gyro = gyro - IntDataVector(512, 512, 512)

        if button is not None:
            button1 = button & 1
            button2 = (button & 2) >> 1
        else:
            button1 = None
            button2 = None

        return robo_pose, raw_gyro, raw_accel, button1, button2

    def switch_motor_power_service_call(self, req):
        return self.cm_730.switch_motor_power(req.power)

    def set_LED_service_call(self, eye, head):
        """ Takes two 3 tuples for RGB"""
        self.led_eye = (eye[0], eye[1], eye[2])
        self.led_head = (head[0], head[1], head[2])
        # always successful
        return True

    def publish_joints(self, robo_pose):
        """
        Sends the Joint States to ROS
        """
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = robo_pose.get_joint_names()
        msg.position = robo_pose.get_positions()
        msg.velocity = robo_pose.get_speeds()
        msg.effort = robo_pose.get_loads()
        self.joint_publisher.publish(msg)

    def publish_additional_servo_data(self, temps, voltages):
        msg = AdditionalServoData()
        temperatures = []
        for temp in temps:
            ros_temp = Temperature()
            ros_temp.header.stamp = rospy.Time.now()
            ros_temp.temperature = temp
            ros_temp.variance = 0
            temperatures.append(ros_temp)
        msg.temperature = temperatures
        msg.voltage = voltages
        self.temp_publisher(msg)

    def publish_imu(self, gyro, accel):
        msg = Imu()
        msg.linear_acceleration = accel
        msg.angular_velocity = gyro
        self.imu_publisher.publish(msg)

    def publish_buttons(self, button1, button2):
        msg = Buttons()
        msg.button1 = button1
        msg.button2 = button2
        self.button_publisher.publish(msg)


if __name__ == "__main__":
    rospy.logdebug("starting cm730 node")
    cm730_node = CM730Node()

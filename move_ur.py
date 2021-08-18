#!/usr/bin/python

# Main script
# Lets the user run the keyboard control or network control (not yet implemented) functions

from Keyboard_Controller import KeyboardMoveJointsGazebo
import math
import numpy as np
from paho.mqtt import client as mqtt_client
from Publishers import move_to_initial_pose
from Publishers import MQTT_pub_sub
from Publishers import ROS_pub_sub
import pygame
from pygameKeyPolling import PygWindow
import Queue
import random
import rospy
from threadClass import NewThread
import time
from trajectory_msgs.msg import JointTrajectoryPoint


def arr_to_str(array):
    string = ''
    for item in array:
        string += str(item)
        string += ' '
    return string


def str_to_arr(string):
    lst = []
    for item in string.split():
        lst.append(float(item))
    return lst


def options_menu():
    options_list = ["k", "n"]
    print("Select the corresponding option:")
    print("k = keyboard control")
    print("n = network control")
    inp = raw_input("Write option: ")
    if inp not in options_list:
        print("Option not available.")
        inp = "exit"
    return inp


def Keyboard_Control():
    print("Keyboard Control selected.")

    # Launching keypolling window
    q = Queue.Queue()
    keypolling_thread = NewThread(1, "Thread-1", PygWindow(refresh_rate=10), "KeyPolling", q)
    keypolling_thread.start()

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                            'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                            'wrist_3_joint']

    # Instantiating publisher-subscriber object
    pub_sub = ROS_pub_sub(node_name='move_joints',
                          pub_topic='/arm_controller/command',
                          sub_topic='/joint_states',
                          joint_names=joint_names,
                          rate = 60,
                          speed=1)

    # Moving to initial position
    desired_pose = [0., -math.pi / 2, 0., -math.pi / 2, 0., 0.] #[0, 0, 0, 0, 0, 0]
    move_to_initial_pose(pub_subs=pub_sub, pose=desired_pose)

    # Keyboard joints control
    KeyboardMoveJointsGazebo(queueInput=q, pub_sub=pub_sub)


def Network_Control():
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                   'wrist_3_joint']

    # Instantiating ROS publisher-subscriber object
    ros_pub_sub = ROS_pub_sub(node_name='move_joints',
                          pub_topic='/arm_controller/command',
                          sub_topic='/joint_states',
                          joint_names=joint_names,
                          rate=60,
                          speed=1)

    # Moving to initial position
    desired_pose = [0., -math.pi / 2, 0., -math.pi / 2, 0., 0.]  # [0, 0, 0, 0, 0, 0]
    move_to_initial_pose(pub_subs=ros_pub_sub, pose=desired_pose)

    # Instantiating MQTT publisher-subscriber object
    mqtt_pub_sub = MQTT_pub_sub()

    last_msg = []
    goal = [0., 0., 0.75]

    while True:
        time.sleep(0.5)

        # Publishing
        msg = np.ones(15)
        msg = ros_pub_sub.current_pos
        msg.extend(ros_pub_sub.current_vel)
        msg.extend(goal)
        msg = arr_to_str(msg)
        mqtt_pub_sub.publish(msg)

        # Subscribing
        mqtt_pub_sub.subscribe()
        print("Subscriber message = {0}".format(str_to_arr(mqtt_pub_sub.sub_msg)))
        sub_msg_list = str_to_arr(mqtt_pub_sub.sub_msg)
        # if sub_msg_list == last_msg:
        #     mqtt_pub_sub.sub_msg = ' '
        # last_msg = sub_msg_list

        if len(sub_msg_list) > 0:
            ros_pub_sub.publish(goal=sub_msg_list)


def main():
    exit = False
    inp = options_menu()
    if inp == "exit":
        exit = True
    if not exit:
        if inp == "k":
            Keyboard_Control()
        if inp == "n":
            Network_Control()
    else:
        print("Exiting.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        print("Program interrupted.")

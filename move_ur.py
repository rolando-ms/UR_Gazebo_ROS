#!/usr/bin/python

# Main script
# Lets the user run the keyboard control or network control functions

import math
import numpy as np
import pygame
import Queue
import random
import rospy
import tf
import time

from Helpers import arr_to_str
from Helpers import move_to_initial_pose
from Helpers import move_to_initial_pose2
from Helpers import str_to_arr
from Keyboard_Controller import KeyboardMoveJointsGazebo
from multipledispatch import dispatch
from Publishers_Subscribers import MQTT_pub
from Publishers_Subscribers import MQTT_pub_sub
from Publishers_Subscribers import MQTT_sub
from Publishers_Subscribers import ROS_pub
from Publishers_Subscribers import ROS_pub_sub
from Publishers_Subscribers import ROS_sub
from Publishers_Subscribers import ROS_tf_listener
from pygameKeyPolling import PygWindow
from threadClass import NewThread



def options_menu():
    options_list = ["k"]
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
                          rate=60,
                          speed=1)

    # Moving to initial position
    desired_pose = [0., -math.pi / 2, 0., -math.pi / 2, 0., 0.] #[0, 0, 0, 0, 0, 0]
    # move_to_initial_pose(pub_subs=pub_sub, pose=desired_pose)

    # Keyboard joints control
    KeyboardMoveJointsGazebo(queueInput=q, pub_sub=pub_sub)



def main():
    exit = False
    inp = options_menu()
    if inp == "exit":
        exit = True
    if not exit:
        if inp == "k":
            Keyboard_Control()
    else:
        print("Exiting.")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        print("Program interrupted.")

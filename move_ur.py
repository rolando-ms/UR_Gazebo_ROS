#!/usr/bin/python

# Main script
# Lets the user run the keyboard control or network control (not yet implemented) functions

import math
from Keyboard_Controller import KeyboardMoveJointsGazebo
from Publishers import ROS_pub_sub
import pygame
from pygameKeyPolling import PygWindow
import Queue
from threadClass import NewThread
from trajectory_msgs.msg import JointTrajectoryPoint
import rospy


# Moves robot to given pose until threshold is reached
def move_to_initial_pose(pub_subs, pose, threshold = 0.01):
    dist = 1e6
    steps = 0
    while(dist > threshold):
        pub_subs.publish(pose)
        current_pos = pub_subs.current_pos #rospy.wait_for_message("joint_states", JointState).position
        dist = euclidean_distance(current_pos, pose)
        # print("current pos = {0}".format(current_pos))
        print("distance = {0}".format(dist))
        steps += 1


def euclidean_distance(current_pos, desired_pos):
    sqr_sum = 0
    for i in range(len(current_pos)):
        sqr_sum += (desired_pos[i] - current_pos[i])**2
    return math.sqrt(sqr_sum)


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
    print("Not implemented.")

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

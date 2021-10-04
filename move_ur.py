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
                          rate=60,
                          speed=1)

    # Moving to initial position
    desired_pose = [0., -math.pi / 2, 0., -math.pi / 2, 0., 0.] #[0, 0, 0, 0, 0, 0]
    # move_to_initial_pose(pub_subs=pub_sub, pose=desired_pose)

    # Keyboard joints control
    KeyboardMoveJointsGazebo(queueInput=q, pub_sub=pub_sub)


def Network_Control():
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                   'wrist_3_joint']

    rate = 60

    # Instantiating ROS publisher-subscriber object
    ros_pub = ROS_pub(node_name='move_joints_sub',
                      pub_topic='/arm_controller/command',
                      joint_names=joint_names,
                      rate=rate)

    # Instantiating ROS publisher-subscriber object
    ros_sub = ROS_sub(node_name='move_joints_sub',
                          sub_topic='/joint_states',
                          joint_names=joint_names,
                          rate=rate)

    # Moving to initial position
    desired_pose = [0., -np.pi / 2, 0., -np.pi / 2, 0., 0.]  # [0, 0, 0, 0, 0, 0]
    # desired_pose = [0.] * 6
    # move_to_initial_pose(pub_subs=ros_pub_sub, pose=desired_pose)
    move_to_initial_pose2(pub=ros_pub, sub=ros_sub, pose=desired_pose)
    print("Initial position reached! Ready to move!")

    # Instantiating MQTT publisher-subscriber object
    # mqtt_pub_sub = MQTT_pub_sub()
    mqtt_pub = MQTT_pub()

    last_msg = []
    goal = [0., 0., 1.75]
    # goal = [0.5, -0.16, 1.93]

    tf_listener = ROS_tf_listener(reference_frame='/world', target_frame='/wrist_3_link')
    # point_in_reference = [0.,0.,0.,1.] # x,y,z,scaling
    base_offset = np.array([0.5, 0., 1.5])

    # Point to transform with its reference frame (Taking the origin of the reference frame)
    tf_listener.BuildPointStamped(point=[0.,0.,0.])

    # Joint position limits
    dof_lower_limits = np.array([-np.pi] * 6)
    dof_upper_limits = np.array([np.pi] * 6)

    # Joint velocity limits
    dof_vel_lower_limits = np.array([-2.16, -2.16, -3.2, -3.2, -3.2, -3.2])
    dof_vel_upper_limits = -1 * dof_vel_lower_limits


    while True:
        time.sleep(1./rate)

        trans = []
        rot = []
        trans, rot = tf_listener.LookUpTransform()
        point_in_new_frame = tf_listener.TransformPoint()

        # Computing target vector
        world_point = np.array([point_in_new_frame.point.x + base_offset[0],
                                point_in_new_frame.point.y + base_offset[1],
                                -1. * (point_in_new_frame.point.z - base_offset[2])])
        world_target = np.array(goal)
        to_target = world_target - world_point

        # Sending observations for inference
        msg = np.ones(15)
        dof_pos = ros_sub.current_pos
        dof_pos = np.array(dof_pos)
        dof_pos = dof_pos.clip(dof_lower_limits, dof_upper_limits)
        dof_pos_scaled = (2.0 * (dof_pos - dof_lower_limits)
                          / (dof_upper_limits - dof_lower_limits) - 1.0)
        msg = dof_pos_scaled.tolist()
        current_vel = np.array(ros_sub.current_vel)
        current_vel = current_vel.clip(dof_vel_lower_limits, dof_vel_upper_limits)
        msg.extend(current_vel * 0.1)
        msg.extend(to_target)
        msg = arr_to_str(msg)
        mqtt_pub.publish(msg)

        # # Receiving inference result
        # mqtt_pub_sub.subscribe()
        # print("Subscriber message = {0}".format(str_to_arr(mqtt_pub_sub.sub_msg)))
        # sub_msg_list = str_to_arr(mqtt_pub_sub.sub_msg)
        #
        # # Updating simulation
        # sub_msg_np = np.array(sub_msg_list)
        # # sub_msg_np*=np.pi
        # if len(sub_msg_list) > 0:
        #     sub_msg_np = ((sub_msg_np + 1) * (dof_upper_limits - dof_lower_limits)) / 2 + dof_lower_limits
        #     ros_pub_sub.publish(goal=sub_msg_np)


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

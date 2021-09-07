#!/usr/bin/python

import math


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


def move_to_initial_pose2(pub, sub, pose, threshold = 0.01):
    dist = 1e6
    steps = 0
    while(dist > threshold):
        pub.publish(pose)
        current_pos = sub.current_pos #rospy.wait_for_message("joint_states", JointState).position
        dist = euclidean_distance(current_pos, pose)
        # print("current pos = {0}".format(current_pos))
        print("distance = {0}".format(dist))
        steps += 1


# Computes the euclidean distance between 2 lists of numbers
def euclidean_distance(current_pos, desired_pos):
    sqr_sum = 0
    for i in range(len(current_pos)):
        sqr_sum += (desired_pos[i] - current_pos[i])**2
    return math.sqrt(sqr_sum)


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
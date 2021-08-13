#!/usr/bin/env/ python
import math
import pygame
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint


jointsMapping = {
    # K_0
    '48': 0,
    # K_1
    '49': 1,
    # K_2
    '50': 2,
    # K_3
    '51': 3,
    # K_4
    '52': 4,
    # K_5
    '53': 5,
    # K_6
    '54': 6,
    # K_7
    '55': 7,
    # K_8
    '56': 8,
    # K_9
    '57': 9,
    # K_q
    '113': 10,
    # K_w
    '119': 11,
    # K_e
    '101': 12,
    # K_r
    '114': 13,
    # K_t
    '116': 14,
    # K_z
    '122': 15,
    # K_u
    '117': 16,
    # K_i
    '105': 17,
    # K_o
    '111': 18,
    # K_p
    '112': 19,
    # K_a
    '97': 20
}


def publish_new_joint_pos(pub_sub, joint, deltaPos):
    new_target = list(pub_sub.current_pos) #list(pub_sub.current_pos)
    round_list(new_target)

    # Clipping value to joint limits
    new_target[joint] += deltaPos #deltaPos
    if(new_target[joint] > 2 * math.pi):
        new_target[joint] = 2 * math.pi
    if(new_target[joint] < -2 * math.pi):
        new_target[joint] = -2 * math.pi

    pub_sub.publish(new_target)


def euclidean_distance(current_pos, desired_pos):
    sqr_sum = 0
    for i in range(len(current_pos)):
        sqr_sum += (desired_pos[i] - current_pos[i])**2
    return math.sqrt(sqr_sum)


def compute_delta_pos(joint, pub_sub):
    # Shoulder joints have a smaller max velocity (120deg/s)
    if joint == 0 or joint == 1:
        # (To rad multiplier) * robot speed * (max joint velocity)
        deltaPos = (math.pi/360) * pub_sub.speed * (120./pub_sub.rate)
    else:
        # (To rad multiplier) * robot speed * (max joint velocity)
        deltaPos = (math.pi / 360) * pub_sub.speed * (180. / pub_sub.rate)
    return deltaPos


def round_list(valueList):
    roundedList = []
    for item in valueList:
        roundedList.append(round(item, 2))
    return roundedList


'''
Function used to operate the UR robot with given keystrokes with the following strategy:

By pressing the "left" or "right" arrow keys you add or subtract (respectively) a given delta theta to the selected
joint. The key functionalities are as follows:

* Spacebar = End program
* Left arrow = Add angle from joint
* Right arrow = Subtract arrow from joint
* 0,1,2,3,4,5,6,7,8,9,q,w,e,r,t,z,u,i,o,p,a = Select joint from 0 to 20 respectively

*** Note: In order to read the key strokes, the black screen from pygame must be active (click on it). Also,
this functions publishes the values using the client
'''

def KeyboardMoveJointsGazebo(queueInput, pub_sub):
    print("Keyboard controller started")
    if(pub_sub.pub_topic == '/arm_controller/command'):
        pub_sub.traj.header.stamp = rospy.Time.now()
    # pts = JointTrajectoryPoint()
    joint = 0
    deltaPosition = math.pi/128.0
    running = True
    while running:
        keys = queueInput.get()
        # Read and perform action
        if keys[pygame.K_SPACE] or keys[pygame.K_ESCAPE]:
            running = False
        if keys.count(1) > 0 and str(keys.index(1)) in jointsMapping:
            if jointsMapping[str(keys.index(1))] < len(pub_sub.joint_names):
                joint = jointsMapping[str(keys.index(1))]
                print("Joint {0} selected".format(joint))
        if keys[pygame.K_LEFT]:
            key = 'Left<<<<<<<<<<<<<'
            # print(key)
            deltaPosition = compute_delta_pos(joint=joint, pub_sub=pub_sub)
            publish_new_joint_pos(pub_sub=pub_sub, joint=joint, deltaPos=deltaPosition)
        if keys[pygame.K_RIGHT]:
            key = 'Right>>>>>>>>>>>>>'
            # print(key)
            deltaPosition = compute_delta_pos(joint=joint, pub_sub=pub_sub)
            publish_new_joint_pos(pub_sub=pub_sub, joint=joint, deltaPos=-deltaPosition)
        if keys[pygame.K_p]:
            pub_sub.info()
        with queueInput.mutex:
            queueInput.queue.clear()
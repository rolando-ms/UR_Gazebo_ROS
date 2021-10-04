#!/usr/bin/python
import actionlib
import math
import numpy as np
import roslib; roslib.load_manifest('ur_driver')
import rospy
import time
from control_msgs.msg import *
from Helpers import move_to_initial_pose
from Helpers import str_to_arr
from Publishers_Subscribers import MQTT_pub_sub
from Publishers_Subscribers import MQTT_sub
from Publishers_Subscribers import ROS_pub
from Publishers_Subscribers import ROS_pub_sub
from trajectory_msgs.msg import *


# client = None

def client_mqtt_publish(client):
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                   'wrist_3_joint']

    rate = 30

    mqtt_sub = MQTT_sub(sub_client_id='DFKImqtt6', sub_username='dfki1', sub_password='dfkipass19',
                            sub_broker='broker.emqx.io', sub_port=1883, sub_topic='topic/goal')

    # Joint position limits
    dof_lower_limits = np.array([-np.pi] * 6)
    dof_upper_limits = np.array([np.pi] * 6)

    # Joint velocity limits
    dof_vel_lower_limits = np.array([-2.16, -2.16, -3.2, -3.2, -3.2, -3.2])
    dof_vel_upper_limits = -1 * dof_vel_lower_limits

    # Joint Targets
    targets = [0., -np.pi / 2, 0., -np.pi / 2, 0., 0.] # Initial targets = initial pose

    speed_scales = np.ones_like(targets)
    speed_scales *= 1
    action_scale = 7.5

    mqtt_sub.subscribe()

    n = 1

    while True:
        # time.sleep(1./rate)

        # Receiving inference result
        # print("Subscriber message = {0}".format(str_to_arr(mqtt_sub.sub_msg)))
        sub_msg_list = str_to_arr(mqtt_sub.sub_msg)

        # Updating simulation
        sub_msg_np = np.array(sub_msg_list)
        # sub_msg_np*=np.pi
        if len(sub_msg_list) > 0:
            # sub_msg_np = ((sub_msg_np + 1) * (dof_upper_limits - dof_lower_limits)) / 2 + dof_lower_limits
            targets += speed_scales * (1./60) * sub_msg_np * action_scale
            # targets = sub_msg_np
            targets = targets.clip(dof_lower_limits, dof_upper_limits)

            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = joint_names
            g.trajectory.points = [
                JointTrajectoryPoint(positions=targets, velocities=[0] * 6, time_from_start=rospy.Duration(1./rate))]
            client.send_goal(g)
            n += 1
            try:
                pass
                # client.wait_for_result()
            except KeyboardInterrupt:
                client.cancel_goal()
                raise


def client_single_publish(client):
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                   'wrist_3_joint']

    rate = 60

    # Joint position limits
    dof_lower_limits = np.array([-np.pi] * 6)
    dof_upper_limits = np.array([np.pi] * 6)

    # Joint velocity limits
    dof_vel_lower_limits = np.array([-2.16, -2.16, -3.2, -3.2, -3.2, -3.2])
    dof_vel_upper_limits = -1 * dof_vel_lower_limits

    # Joint Targets
    targets = [0., -np.pi / 2, 0., -np.pi / 2, 0., 0.] # Initial targets = initial pose

    speed_scales = np.ones_like(targets)
    speed_scales *= 1
    action_scale = 7.5

    n = 1

    path_to_file = '/home/rolando/Dokumente/RL/ROS_Gazebo/Tests/actionData.txt'
    file = open(path_to_file, 'r')
    lines = file.readlines()
    lineCount = 0

    for line in lines:
        # time.sleep(1./rate)

        sub_msg_list = str_to_arr(line)

        # Updating simulation
        sub_msg_np = np.array(sub_msg_list)
        # sub_msg_np*=np.pi
        if len(sub_msg_list) > 0:
            # sub_msg_np = ((sub_msg_np + 1) * (dof_upper_limits - dof_lower_limits)) / 2 + dof_lower_limits
            targets += speed_scales * (1./60) * sub_msg_np * action_scale
            # targets = sub_msg_np
            targets = targets.clip(dof_lower_limits, dof_upper_limits)

            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = joint_names
            g.trajectory.points = [
                JointTrajectoryPoint(positions=targets, velocities=[0] * 6, time_from_start=rospy.Duration(1./rate))]
            client.send_goal(g)
            n += 1
            try:
                pass
                # client.wait_for_result()
            except KeyboardInterrupt:
                client.cancel_goal()
                raise


def client_all_publish(client):
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                   'wrist_3_joint']

    rate = 60

    # Joint position limits
    dof_lower_limits = np.array([-np.pi] * 6)
    dof_upper_limits = np.array([np.pi] * 6)

    # Joint velocity limits
    dof_vel_lower_limits = np.array([-2.16, -2.16, -3.2, -3.2, -3.2, -3.2])
    dof_vel_upper_limits = -1 * dof_vel_lower_limits

    # Joint Targets
    targets = [0., -np.pi / 2, 0., -np.pi / 2, 0., 0.] # Initial targets = initial pose

    speed_scales = np.ones_like(targets)
    speed_scales *= 1
    action_scale = 7.5

    n = 1

    path_to_file = '/home/rolando/Dokumente/RL/ROS_Gazebo/Tests/actionData.txt'
    file = open(path_to_file, 'r')
    lines = file.readlines()
    lineCount = 0
    trajectory = []

    for line in lines:
        sub_msg_np = np.array(str_to_arr(line))
        targets += speed_scales * (1. / 60) * sub_msg_np * action_scale
        targets = targets.clip(dof_lower_limits, dof_upper_limits)
        trajectory.append(targets)


    # time.sleep(1./rate)

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = joint_names
    for point in trajectory:
        g.trajectory.points = [
            JointTrajectoryPoint(positions=point, velocities=[0] * 6, time_from_start=rospy.Duration(n/rate))]
        n += 1
    client.send_goal(g)
    try:
        pass
        # client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise



def connect_to_client():
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        return client
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


def main(client):
    # client_mqtt_publish(client)
    # client_single_publish(client)
    client_all_publish(client)


if __name__ == '__main__':
    try:
        client = connect_to_client()
        main(client)
    except rospy.ROSInterruptException, KeyboardInterrupt:
        print("Program interrupted.")

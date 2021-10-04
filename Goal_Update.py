#!/usr/bin/python

import math
import numpy as np
import rospy
import time

from Helpers import arr_to_str
from Helpers import move_to_initial_pose
from Helpers import str_to_arr
from Publishers_Subscribers import MQTT_pub_sub
from Publishers_Subscribers import MQTT_sub
from Publishers_Subscribers import ROS_pub
from Publishers_Subscribers import ROS_pub_sub



def main():
    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                   'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                   'wrist_3_joint']

    rate = 30

    # Instantiating ROS publisher-subscriber object
    ros_pub = ROS_pub(node_name='move_joints_pub',
                      pub_topic='/arm_controller/command',
                      joint_names=joint_names,
                      rate=rate)

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

    while True:
        time.sleep(1./rate)

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
            # print(arr_to_str(targets))
            ros_pub.publish(goal=targets)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException, KeyboardInterrupt:
        print("Program interrupted.")

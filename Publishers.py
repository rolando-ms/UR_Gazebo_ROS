#!/usr/bin/env/ python

# Publisher-Subscriber class for ROS
# This class is used to initialize, publish and subscribe to ROS topics to interface with the UR robot in gazebo

import rospy
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class ROS_pub_sub:
    def __init__(self, node_name, pub_topic, sub_topic, joint_names, rate, speed=0.25):
        self.node_name = node_name
        self.ros_node = rospy.init_node(node_name)
        self.pub_topic = pub_topic
        self.sub_topic = sub_topic
        self.joint_names = joint_names
        self.rate = rate
        self.ros_rate = rospy.Rate(rate)
        self.traj = None
        topic_type = None
        if(pub_topic == '/arm_controller/command'):
            topic_type = JointTrajectory
            self.init_pub_msg()
        if(pub_topic == '/joint_group_position_controller/command'):
            topic_type = Float64MultiArray
        self.pub = rospy.Publisher(pub_topic,
                                   topic_type,
                                   queue_size=1)
        self.current_pos = []
        self.current_vel = []
        self.sub = rospy.Subscriber(sub_topic,
                                    JointState,
                                    self.callback,
                                    queue_size=10)
        self.speed = speed

    # Initializing trajectory message for topic: '/arm_controller/command'
    def init_pub_msg(self):
        self.traj = JointTrajectory()
        self.traj.header = Header()
        self.traj.joint_names = self.joint_names

    # Subscriber callback to retrieve the data.
    # Data in '/joint_states' topic is ordered alphabetically, thus need reordering
    def callback(self, data):
        #rospy.loginfo("I heard %s", data.position)
        self.current_pos = list(data.position)
        self.current_vel = list(data.velocity)
        # Swapping joint 0 value for joint 2 value (elbow and shoulder_pan respectively)
        aux = self.current_pos[0]
        self.current_pos[0] = self.current_pos[2]
        self.current_pos[2] = aux
        aux = self.current_vel[0]
        self.current_vel[0] = self.current_vel[2]
        self.current_vel[2] = aux

    def info(self):
        print("Node name = {0}".format(self.node_name))
        print("Publisher topic = {0}".format(self.pub_topic))
        print("Subscriber topic = {0}".format(self.sub_topic))
        print("Joint names = {0}".format(self.joint_names))
        print("Rate (Hz) = {0}".format(self.rate))
        print("Current joint positions (rad) = {0}".format(self.current_pos))
        print("Current joint velocities (rad/s) = {0}".format(self.current_vel))

    def publish(self, goal):
        if (self.pub_topic == '/arm_controller/command'):
            self.publish_arm_controller_point(point=goal)
        if (self.pub_topic == '/joint_group_position_controller/command'):
            self.publish_joint_group_pos(pos=goal)

    # Publishing trajectory points ('/arm_controller/command')
    def publish_arm_controller_point(self, point):
        new_point = JointTrajectoryPoint()
        new_point.positions = point
        new_point.time_from_start = rospy.Duration((1. / (self.rate)))  # 1./(self.rate)
        self.traj.points = []
        self.traj.points.append(new_point)
        self.traj.header.stamp = rospy.Time.now()
        self.pub.publish(self.traj)
        self.ros_rate.sleep()

    # Publishing point ('/joint_group_position_controller/command')
    def publish_joint_group_pos(self, pos):
        new_point = Float64MultiArray()
        new_point.data = pos
        self.pub.publish(new_point)
        self.ros_rate.sleep()

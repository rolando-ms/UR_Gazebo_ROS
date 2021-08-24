#!/usr/bin/env/ python

from geometry_msgs.msg import PointStamped
import math
from paho.mqtt import client as mqtt_client
import rospy
from std_msgs.msg import Header, Float64MultiArray
from sensor_msgs.msg import JointState
import tf
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Publisher-Subscriber class for ROS
# This class is used to initialize, publish and subscribe to ROS topics to interface with the UR robot in gazebo
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


# Publisher-Subscriber class for MQTT
# This class is used to initialize, publish and subscribe to MQTT topics.
class MQTT_pub_sub:
    def __init__(self, pub_client_id='DFKImqtt1', pub_username='dfki1', pub_password='dfkipass19',
                 pub_broker='broker.emqx.io', pub_port=1883, pub_topic='topic/state',
                 sub_client_id='DFKImqtt2', sub_username='dfki1', sub_password='dfkipass19',
                 sub_broker='broker.emqx.io', sub_port=1883, sub_topic='topic/goal'):
        self.pub_topic = pub_topic
        self.publisher = self.connect_mqtt(client_id=pub_client_id, username=pub_username, password=pub_password,
                                 broker=pub_broker,
                                 port=pub_port)
        self.publisher.loop_start()
        self.sub_topic = sub_topic
        self.sub_msg = ' '
        self.subscriber = self.connect_mqtt(client_id=sub_client_id, username=sub_username, password=sub_password,
                                  broker=sub_broker,
                                  port=sub_port)
        self.subscriber.loop_start()

    def connect_mqtt(self, client_id, username, password, broker, port):
        def on_connect(client, userdata, flags, rc):
            if rc == 0:
                print("Connected to MQTT Broker!")
            else:
                print("Failed to connect, return code %d\n", rc)

        client = mqtt_client.Client(client_id)
        client.username_pw_set(username, password)
        client.on_connect = on_connect
        client.connect(broker, port)
        return client

    def publish(self, msg):
        result = self.publisher.publish(self.pub_topic, msg)
        status = result[0]
        if status == 0:
            print("Send {0} to topic {1}".format(msg, self.pub_topic))
        else:
            print("Failed to send message to topic {0}".format(0))

    def subscribe(self):
        # Callback function for subscriber (Used to retrieve the data)
        def on_message(client, userdata, msg):
            print("Received `{0}` from `{1}` topic".format(msg.payload.decode(), msg.topic))
            self.sub_msg = msg.payload.decode()
            # print(type(self.sub_msg))
            # print(list(self.sub_msg))
        self.subscriber.subscribe(self.sub_topic)
        self.subscriber.on_message = on_message


# ROS tf listener
# This class is used to retrieve transformations from robot model.
class ROS_tf_listener:
    def __init__(self, reference_frame, target_frame):
        self.reference_frame = reference_frame
        self.target_frame = target_frame
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(self.target_frame, self.reference_frame, rospy.Time(0), rospy.Duration(4.0))
        self.translation = []
        self.rotation = []

    def LookUpTransform(self):
        try:
            (self.translation, self.rotation) = self.listener.lookupTransform(self.target_frame, self.reference_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return self.translation, self.rotation

    def TransformPoint(self):
        try:
            point_in_new_frame = self.listener.transformPoint(self.target_frame, self.point_stamped)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        return point_in_new_frame

    def BuildPointStamped(self, point):
        # Point to transform with its reference frame
        self.point_stamped = PointStamped()
        self.point_stamped.header.frame_id = self.reference_frame
        self.point_stamped.header.stamp = rospy.Time(0)
        self.point_stamped.point.x = point[0]
        self.point_stamped.point.y = point[1]
        self.point_stamped.point.z = point[2]

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


# Computes the euclidean distance between 2 lists of numbers
def euclidean_distance(current_pos, desired_pos):
    sqr_sum = 0
    for i in range(len(current_pos)):
        sqr_sum += (desired_pos[i] - current_pos[i])**2
    return math.sqrt(sqr_sum)
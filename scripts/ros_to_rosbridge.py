#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import for ROS bridge
import rospy
from roslibpy import Message, Ros, Topic
import rospkg
from rospy_message_converter import message_converter

# import message
from geometry_msgs.msg import Twist

import time
import json

class rosToRosbridge():
    def __init__(self):
        # ros bridge
        self.bridge_pub = {}
        self.bridge_ros_client = Ros(rospy.get_param('~remote_host','127.0.0.1'), rospy.get_param('~remote_port', 9090))
        self.topicname = '/cmd_vel'
        datatype = 'geometry_msgs/Twist'
        self.bridge_pub[self.topicname] = Topic(self.bridge_ros_client, self.topicname+'_web', datatype)

        # read config
        rospack = rospkg.RosPack()
        conf = json.load(open(rospack.get_path('ros_to_rosbridge')+'/conf/config.json'))

        # ROS sub
        self.cmdvel_sub = rospy.Subscriber(self.topicname, Twist, self.ros_callback, callback_args=self.topicname)
        self.cmdvel_sub = rospy.Subscriber(self.topicname, Twist, self.ros_callback, callback_args=self.topicname)

        self.bridge_ros_client.run_forever()

    def ros_callback(self, message, topicname):
        dict_message = message_converter.convert_ros_message_to_dictionary(message)
        rospy.loginfo(topicname)
        rospy.loginfo(dict_message)
        self.bridge_pub[topicname].publish(Message(dict_message))

if __name__ == '__main__':
    rospy.init_node('ros_to_rosbridge_demo')

    time.sleep(1)
    node = rosToRosbridge()

    while not rospy.is_shutdown():
        try:
            rospy.sleep(0.1)
        except KeyboardInterrupt:
            print("Stop by Ctrl+C")
            break
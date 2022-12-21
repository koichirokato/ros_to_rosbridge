#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from roslibpy import Ros, Topic
import rospkg
from rospy_message_converter import message_converter
import rospy
import importlib

import yaml
import re
from functools import partial

class rosbridge_to_ros():
    def __init__(self):
        # ROS bridge
        self.bridge_sub   = {}
        self.ros_pub  = {}
        rospack = rospkg.RosPack()
        self.conf = yaml.safe_load(open(rospy.get_param('~config_file', '../conf/config.yaml')))

        self.use_id_for_ns = bool(rospy.get_param('~use_id_for_ns', "False"))

        self.bridge_ros_client = Ros(rospy.get_param('~remote_host','127.0.0.1'), rospy.get_param('~remote_port', 9090))

        rospy.loginfo('')
        rospy.loginfo('Remote Host : [%s:%s]', rospy.get_param('~remote_host','127.0.0.1'), rospy.get_param('~remote_port', 9090))
        rospy.loginfo('')

        rospy.loginfo('Make below topics bridge')

        if self.conf['include_topics']==None:
            # if match exclude topic => flag is True
            self.bridge_ros_client.get_topics(callback=self.topics_callback)
            # For some reason the blocking version of this call does not return, so we need to use the callback version
            # Execution continues immediately in topics_callback
        else:
            for topic_conf in self.conf['include_topics']:
                self.create_pub_sub(topic_conf['name'], topic_conf['type'])


    def topics_callback(self, topics):
        for i, topic in enumerate(topics['topics']):
            # Check if this topic is in the list of topics to exclude
            if len(list(filter(lambda ex_topic: ex_topic == topic, self.conf['exclude_topics']))) == 0:
                self.create_pub_sub(topic, topics['types'][i])


    # set bridge subscriber & publisher
    def create_pub_sub(self, topicname, typename):
        pub_topicname =  ('/' + self.conf['id'] if self.use_id_for_ns else '') + topicname
        rospy.loginfo('Bridge Sub:[%s] => Local Pub:[%s]', topicname, pub_topicname)

        type_class = getattr(importlib.import_module(typename.split('/')[0]+'.msg'), typename.split('/')[1])
        self.bridge_sub[topicname] = Topic(self.bridge_ros_client, topicname, typename)
        self.ros_pub[topicname] = rospy.Publisher(pub_topicname, type_class, queue_size=10) #ToDo Queue length as parameter

        callback = partial(self.callback, self.ros_pub[topicname], )
        self.bridge_sub[topicname].subscribe(callback)

    # Subscribe ROS bridge and publish ROS message
    def callback(self, pub, message):
        s = str(pub.data_class).split('\'')[1].split('.')
        ros_message = message_converter.convert_dictionary_to_ros_message(s[0]+"/"+s[3], message)
        pub.publish(ros_message)

    def run(self):
        self.bridge_ros_client.run_forever()


if __name__ == '__main__':
    rospy.init_node('rosbridge_to_ros')
    node = rosbridge_to_ros()
    node.run()

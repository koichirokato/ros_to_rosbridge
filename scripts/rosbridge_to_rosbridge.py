#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from roslibpy import Message, Ros, Topic
import rospkg

import time
import json
from functools import partial

class testNode():
    def __init__(self):
        # ROS bridge
        self.local_sub = {}
        self.bridge_pub = {}
        rospack = rospkg.RosPack()
        conf = json.load(open(rospack.get_path('ros_to_rosbridge')+'/conf/config.json'))

        self.local_ros_client  = Ros(rospy.get_param('~local_host','127.0.0.1'), rospy.get_param('~local_port', 9090))
        self.bridge_ros_client = Ros(rospy.get_param('~remote_host','127.0.0.1'), rospy.get_param('~remote_port', 9090))
        
        topics = rospy.get_published_topics('/')

        # set bridge subscriber & publisher
        def set_pub_sub(topicname, datatype):
            rospy.loginfo([topicname, datatype])
            self.local_sub[topicname]  = Topic(self.local_ros_client, topicname, datatype)
            self.bridge_pub[topicname] = Topic(self.bridge_ros_client, topicname+'_web', datatype)

            callback = partial(self.callback, self.bridge_pub[topicname])
            self.local_sub[topicname].subscribe(callback)

        for topic in topics:
            if topic[0] in conf['exclude_topics']:
                pass
            else:
                set_pub_sub(topic[0], topic[1])
        
        for topic_conf in conf['include_topics']:
            set_pub_sub(topic_conf['name'].decode(), topic_conf['type'].decode())

        # self.ros_client.on_ready(self.start_thread, run_in_thread=True)
        self.bridge_ros_client.run_forever()

    # Subscribe ROS bridge and publish ROS message by ROS bridge
    def callback(self, pub, message):
        rospy.loginfo(message)
        # ROS callback
        if self.ros_client.is_connected:
            pub.publish(message)
        else:
            rospy.loginfo('Disconnected')

if __name__ == '__main__':
    rospy.init_node('rosbridge_to_rosbridge')

    time.sleep(1)
    node = testNode()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
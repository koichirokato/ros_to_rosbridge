#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import warnings
# warnings.filterwarnings('ignore')

import rospy
from roslibpy import Ros, Topic
import rospkg

import time
import yaml
import re
from functools import partial

class rosbridge_to_rosbridge():
    def __init__(self):
        # ROS bridge
        self.local_sub   = {}
        self.bridge_pub  = {}
        rospack          = rospkg.RosPack()
        conf             = yaml.safe_load(open(rospack.get_path('ros_to_rosbridge')+'/conf/config.yaml'))
        topics           = rospy.get_published_topics('/')
        topics_list_dict = []
        for topic in topics:
            topics_list_dict.append({'name':topic[0], 'type':topic[1]})
        use_id_for_ns   = bool(rospy.get_param('~use_id_for_ns', "False"))

        self.local_ros_client  = Ros(rospy.get_param('~local_host','127.0.0.1'), rospy.get_param('~local_port', 9090))
        self.bridge_ros_client = Ros(rospy.get_param('~remote_host','127.0.0.1'), rospy.get_param('~remote_port', 9090))

        rospy.loginfo('')
        rospy.loginfo('Connect Local Host : [%s:%s], Remote Host : [%s:%s]', rospy.get_param('~local_host','127.0.0.1'), rospy.get_param('~local_port', 9090),rospy.get_param('~remote_host','127.0.0.1'), rospy.get_param('~remote_port', 9090))
        rospy.loginfo('')

        rospy.loginfo('Make below topics bridge')
        
        # set bridge subscriber & publisher
        def set_pub_sub(topicname, datatype):
            pub_topicname = topicname
            if use_id_for_ns: pub_topicname = '/' + conf['id'] + topicname            
            rospy.loginfo('Local Sub:[%s] => Bridge Pub:[%s]', topicname, pub_topicname)

            self.local_sub[topicname]  = Topic(self.local_ros_client, topicname, datatype)
            self.bridge_pub[topicname] = Topic(self.bridge_ros_client, pub_topicname, datatype)

            callback = partial(self.callback, self.bridge_pub[topicname])
            self.local_sub[topicname].subscribe(callback)


        if conf['include_topics']==None:
            # if match exclude topic => flag is True
            flag = False
            for topic_dict in topics_list_dict:
                for e_topic in conf['exclude_topics']:
                    flag = bool(re.match(e_topic, topic_dict['name']))
                    if flag:
                        break
                if not flag:
                    
                    set_pub_sub(topic_dict['name'], topic_dict['type'])
        else:
            for topic_conf in conf['include_topics']:
                set_pub_sub(topic_conf['name'].decode(), topic_conf['type'].decode())

        try:
            self.bridge_ros_client.run_forever()
        except KeyboardInterrupt:
            self.bridge_ros_client.close()

    # Subscribe ROS bridge and publish ROS message by ROS bridge
    def callback(self, pub, message):
        # ROS callback
        if self.bridge_ros_client.is_connected:
            pub.publish(message)
        else:
            rospy.loginfo('Disconnected')

if __name__ == '__main__':
    rospy.init_node('rosbridge_to_rosbridge')

    time.sleep(1)
    node = rosbridge_to_rosbridge()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
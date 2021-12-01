# ros_to_rosbridge
This is the system with mutli roscore using ros bridge.  

If you use developed ros node, you can
 - communicate multiple machines with multiple ros master
   (also can do using WSL2)
 - publish any topics
 - distinguish topicname when you publish to another ros master
   (it is possible to leave it unchanged)
   example:
    〇 /cmd_vel => /cmd_vel
    〇 /cmd_vel => /robot_a/cmd_vel

![image.png](https://qiita-image-store.s3.ap-northeast-1.amazonaws.com/0/426354/f30ff6c2-dca7-da87-62f5-ed6ba5af58fc.png)

# Dependencies
[ROS bridge](http://wiki.ros.org/rosbridge_suite), [roslibpy](https://roslibpy.readthedocs.io/en/latest/), [rospy-message-converter](http://wiki.ros.org/rospy_message_converter)

```shell
$ sudo apt-get install ros-<rosdistro>-rosbridge-server
$ pip install roslibpy
$ sudo apt-get install ros-<rosdistro>-rospy-message-converter
```

# launch
If you use 2 machines, you have to do both machines.

## ROS bridge websocket server
```shell
$ roslaunch rosbridge_server rosbridge_websocket.launch
```

## ROS to ROS Bridge (this node)
```shell
$ roslaunch ros_to_rosbridge rosbridge_to_rosbridge.launch remote_host:={remote PC ip address}
```

| param name | default | explanation |
|:-:|:-:|:-:|
| local_host | 127.0.0.1 | Local address using Websocket server |
| local_port | 9090 |  Local port using Websocket server |
| remote_host | 127.0.0.1 | Remote address using Websocket server |
| remote_port | 9090 | Local port using Websocket server |
| use_id_for_ns | false | Add id in config file to topicname (If true, /cmd_vel => /id/cmd_vel) |

## Config
In `ros_to_rosbridge/conf`, there is `config.yaml`.  
Default is below.  

```text:ros_to_rosbridge/conf/config.yaml
id : robot_a
exclude_topics:
  - /rosout
  - /rosout_agg
  - /client_count
  - /connected_clients
  # - /turtle1/*
include_topics:
  # - name : /cmd_vel 
  #   type : geometry_msgs/Twist
  # - name : /odom
  #   type : nav_msgs/Odometry
```

| Item | Explanation |
|:-:|:-:|
| id | id for topicname when bridge |
| exclude_topics | Topics for which bridges are excluded |
| include_topics | The topic to be bridged (topic name and data type) |

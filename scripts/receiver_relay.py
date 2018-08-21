#!/usr/bin/python2

from __future__ import print_function

import rospy
import std_msgs
import roslibpy
from multi_robot_relay.msg import(
    MultiBotSignal,
    MultiBotTalking,
)
import alloy 

class ReceiverRelay():

    def __init__(self, hostname, port):
        #hostname and IP
        self._hostname = hostname
        self._port = port

        self._python_ros = roslibpy.Ros(host=self._hostname, port=self._port)
        self._connected_robot_id = ""

        self._python_ros.on_ready(lambda: print('Is ROS connected?', self._python_ros.is_connected)) 
        #self._python_ros.on_ready(self.connect,True)       
      
        self._multibot_signal_pub = rospy.Publisher('/multibot_local/signal',MultiBotSignal,queue_size=1)
        self._multibot_talking_pub = rospy.Publisher('/multibot_local/talking',MultiBotTalking,queue_size=1)

    def connect(self):
        #listen for the topis that are multibot related
        self._id_topic = roslibpy.Topic(self._python_ros, name='/multibot_relay/robot_id', message_type="std_msgs/String",queue_size=10)
        self._id_topic.subscribe(self._id_callback)
        self._signal_topic = roslibpy.Topic(self._python_ros, name='/multibot_relay/signal', message_type="std_msgs/String",queue_size=10)
        self._signal_topic.subscribe(self._signal_callback)
        self._talking_topic = roslibpy.Topic(self._python_ros, name='/multibot_relay/talking', message_type="std_msgs/String",queue_size=10)
        self._id_topic.subscribe(self._talking_callback)


    def spin(self):
        self._python_ros.run_forever()



    def _id_callback(self, msg):
        self._connected_robot_id = msg.data

    def _signal_callback(self, msg):
        print("signal callback")
        #create header
        relay_msg = MultiBotSignal()
        relay_msg.id = self._connected_robot_id
        relay_msg.header = alloy.ros.create_ros_header(rospy)
        relay_msg.signal = msg.data
        #publish that
        rospy.loginfo('received signal {} form {}'.format(relay_msg.signal,relay_msg.id))
        self._multibot_signal_pub.publish(relay_msg)

    def _talking_callback(self, msg):
        #create header
        relay_msg = MultiBotTalking()
        relay_msg.id = self._connected_robot_id
        relay_msg.header = alloy.ros.create_ros_header(rospy)
        relay_msg.signal = msg.data
        #publish that
        self._multibot_talking_pub.publish(relay_msg)

def main():
    rospy.init_node('receiver_relay')
    receiver = ReceiverRelay(hostname='192.168.0.200', port=9090)
    receiver.connect()
    rospy.loginfo('Receiver Relay Started')
    receiver.spin()

if __name__ == '__main__':
    main()
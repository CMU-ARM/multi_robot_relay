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
import signal
import threading

class ReceiverRelay():

    def __init__(self, reconnect_on_fail=True):
        #hostname and IP
        self._hostname = rospy.get_param('/receiver/hostname', 'localhost')
        self._port = rospy.get_param('/receiver/port', 9090)

        self._python_ros = roslibpy.Ros(host=self._hostname, port=self._port)
        self._connected_robot_id = ""


        self._python_ros.on_ready(lambda: rospy.loginfo('Connected to external ROS on {}:{}'.format(self._hostname, self._port)))
        self._python_ros.on_ready(self.connect,True)       
      
        self._python_ros.on('close',self._reconnection_protocol)
        self._restart_notification_flag = False
        self._keep_alive = reconnect_on_fail

        self._multibot_signal_pub = rospy.Publisher('/multibot_local/signal', MultiBotSignal, queue_size=10)
        self._multibot_talking_pub = rospy.Publisher('/multibot_local/talking', MultiBotTalking, queue_size=1)
        rospy.loginfo('initialization complete')

    def _reconnection_protocol(self, msg):
        rospy.loginfo('Receiver lost connection to external ROS master at {}.'.format(self._hostname))
        
        #There's a bug where the library doesn't try to reconnect
        #we just going to hack it by recreating another instance & replacing it
        #I tried to figure out how to reconnect, but it's not working correctly
        if self._keep_alive:
            rospy.loginfo('Attempting reconnect to external ROS master at {}.'.format(self._hostname))
            self._python_ros = roslibpy.Ros(host=self._hostname, port=self._port)
            self._python_ros.on_ready(lambda: rospy.loginfo('Reconnected to external ROS on {}:{}'.format(self._hostname, self._port)))
            self._python_ros.on_ready(self.connect,True)       
            self._python_ros.on('close',self._reconnection_protocol)


    def connect(self):
        #listen for the topis that are multibot related
        self._id_topic = roslibpy.Topic(self._python_ros, name='/multibot_relay/robot_id', message_type="std_msgs/String",queue_size=10)
        self._id_topic.subscribe(self._id_callback)

        self._signal_topic = roslibpy.Topic(self._python_ros, name='/multibot_relay/signal',
            message_type="multi_robot_relay/MultiBotSignal",queue_size=5)
        self._signal_topic.subscribe(self._signal_callback)

        self._talking_topic = roslibpy.Topic(self._python_ros, name='/multibot_relay/talking', 
            message_type="multi_robot_relay/MultiBotTalking",queue_size=5)
        self._talking_topic.subscribe(self._talking_callback)

        rospy.loginfo('Receiver Relay Started')
        


    def stop(self):
        self._keep_alive = False
        self._python_ros.terminate()
        rospy.loginfo('closing connection to {}:{}'.format(self._hostname, self._port))

    def spin(self):
        self._python_ros.run_forever()


    def _id_callback(self, msg):
        #print(msg)
        self._connected_robot_id = msg.data


    def _signal_callback(self, msg):
        ori_msg = MultiBotSignal()
        ori_msg.header.frame_id = msg['header']['frame_id']
        ori_msg.header.seq = msg['header']['seq']
        ori_msg.header.stamp.nsecs = msg['header']['stamp']['nsecs']
        ori_msg.header.stamp.secs = msg['header']['stamp']['secs']

        ori_msg.signal = msg['signal']
        ori_msg.id = msg['id']

        #convert back to normal message
        self._multibot_signal_pub.publish(ori_msg)



    def _talking_callback(self, msg):
        rospy.logwarn('TALKING NOT FULLY IMPLEMENTED')
        self._multibot_talking_pub.publish(msg)

def main():
    rospy.init_node('receiver_relay')
    receiver = ReceiverRelay()

    def ctrl_handler(sig, frame):
        receiver.stop()
    signal.signal(signal.SIGINT, ctrl_handler)

    receiver.spin()

if __name__ == '__main__':
    main()
#!/usr/bin/python2


import rospy
import std_msgs
from multi_robot_relay.msg import(
    MultiBotSignal,
    MultiBotTalking,
)

def main():
    rospy.init_node('sender_relay')

    robot_id = rospy.get_param('sender/robot_id', 'DEFAULT_ROBOT_ID')
    robot_id_msgs = std_msgs.msg.String()
    robot_id_msgs.data = robot_id

    #publish some useful internal information
    id_pub = rospy.Publisher('multibot_relay/robot_id',std_msgs.msg.String, queue_size=1)
    signal_pub = rospy.Publisher('multibot_relay/signal', MultiBotSignal, queue_size=1)
    talking_pub = rospy.Publisher('multibot_relay/talking', MultiBotTalking, queue_size=1)

    def signal_callback(msg):
        msg.id = robot_id
        signal_pub.publish(msg)

    def talking_callback(msg):
        msg.id = robot_id
        talking_pub.publish(msg)

    #listen for the local stuff
    signal_topic = rospy.Subscriber('multibot_sender/signal', MultiBotSignal, signal_callback ,queue_size=10)
    talking_topic = rospy.Subscriber('multibot_sender/talking', MultiBotTalking, talking_callback ,queue_size=10)


    rate = rospy.Rate(1)
    rospy.loginfo('Starting Sender Relay')
    while not rospy.is_shutdown():
        id_pub.publish(robot_id_msgs)
        rate.sleep()

if __name__ == '__main__':
    main()
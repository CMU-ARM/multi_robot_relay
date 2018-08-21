#!/usr/bin/python2


import rospy
import std_msgs

def main():
    rospy.init_node('sender_relay')

    #publish some useful internal information
    id_pub = rospy.Publisher('multibot_relay/robot_id',std_msgs.msg.String, queue_size=1)
    signal_pub = rospy.Publisher('multibot_relay/signal',std_msgs.msg.String, queue_size=1)
    talking_pub = rospy.Publisher('multibot_relay/talking',std_msgs.msg.String, queue_size=1)

    robot_id = "H1"
    robot_id_msgs = std_msgs.msg.String()
    robot_id_msgs.data = robot_id

    rate = rospy.Rate(1)
    rospy.loginfo('Starting Sender Relay')
    while not rospy.is_shutdown():
        id_pub.publish(robot_id_msgs)
        rate.sleep()

if __name__ == '__main__':
    main()
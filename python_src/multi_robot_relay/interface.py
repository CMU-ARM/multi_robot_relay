

import rospy
from multi_robot_relay.msg import(
    MultiBotSignal,
    MultiBotTalking,
)
import time
import threading
import std_msgs
import alloy.ros
import copy


class MultiBotInterface():

    def __init__(self):
        #subscribe to all topics that locals does

        rospy.Subscriber('multibot_local/signal', MultiBotSignal, self._signal_callback, queue_size=5)
        rospy.Subscriber('multibot_local/talking', MultiBotTalking,self._talking_callback,queue_size=5)

        self._signal_pub = rospy.Publisher('multibot_sender/signal', MultiBotSignal, queue_size=1)
        self._talking_pub = rospy.Publisher('multibot_sender/talking', MultiBotTalking, queue_size=1)

        self._signal_event = threading.Event()
        self._signal_waiting_id = ""
        self._signal_data = ""

        self._last_signal = ""

        self._signal_waiting_list = []


    def _signal_callback(self, msg):
        self._last_signal = msg.signal
        if len(self._signal_waiting_list) > 0:
            for waiting_signal_info in list(self._signal_waiting_list):
                print(waiting_signal_info)
                if waiting_signal_info['id'] == "" or msg.id == waiting_signal_info['id']:
                    if waiting_signal_info['signal'] == msg.signal:
                        #this is the signal we are waiting for
                        waiting_signal_info['event'].set()
                        #remove the signal object
                        self._signal_waiting_list.remove(waiting_signal_info)
                if waiting_signal_info['signal'] == "" and waiting_signal_info['id'] == msg.id:
                    #this is the signal we are waiting for
                    waiting_signal_info['event'].set()
                    #remove the signal object
                    self._signal_waiting_list.remove(waiting_signal_info)                    


    def _talking_callback(self, msg):
        pass

    def send_signal(self, signal):
        #create the message
        msg = MultiBotSignal()
        msg.header = alloy.ros.create_ros_header(rospy)
        msg.signal = signal

        rospy.loginfo('publishing {}'.format(signal))
        #publish at a 10Hz for 1 second
        rate = rospy.Rate(10)
        for x in range(0,10):
            self._signal_pub.publish(msg)
            rate.sleep()

    def talking(self):
        msg = MultiBotTalking()
        #publish at a 10Hz for 1 second
        rate = rospy.Rate(10)
        for x in range(0,10):
            self._talking_pub.publish(msg)
            rate.sleep()

    def wait_for_robot(self, _id, duration=None):
        #create the object
        event_obj = threading.Event()
        signal_event = {
            'id':_id,
            'signal':"",
            'event':event_obj
        }
        self._signal_waiting_list.append(signal_event)

        #wait for the event
        rospy.loginfo('waiting for signal from {}'.format(_id))
        called = event_obj.wait(duration)
        last_signal = copy.deepcopy(self._last_signal)
        if not called:
            #this means it timed out and we haven't got a callback yet
            self._signal_waiting_list.remove(signal_event)
            return None
        else:
            return last_signal       

    def wait_for_signal(self, signal, _id="", duration=None):
        """Wait for a signal from the given robot. Return true if signal is given
        NOT THREAD SAFE
        """
        
        #create the object
        event_obj = threading.Event()
        signal_event = {
            'id':_id,
            'signal':signal,
            'event':event_obj
        }
        self._signal_waiting_list.append(signal_event)

        #wait for the event
        rospy.logdebug('waiting for signal {} from {}'.format(signal, _id))
        called = event_obj.wait(duration)
        if not called:
            #this means it timed out and we haven't got a callback yet
            self._signal_waiting_list.remove(signal_event)
        #return whether it was success
        return called

def main():
    rospy.init_node('test')
    face = MultiBotInterface()
    face.talking()
    face.send_signal('move')
    rospy.spin()

if __name__ == '__main__':
    main()


import rospy
from multi_robot_relay.msg import(
    MultiBotSignal,
    MultiBotTalking,
)
import time
import threading
import std_msgs


class MultiBotInterface():

    def __init__(self):
        #subscribe to all topics that locals does

        rospy.Subscriber('multibot_local/signal', MultiBotSignal, self._signal_callback, queue_size=5)
        rospy.Subscriber('multibot_local/talking', MultiBotTalking,self._talking_callback,queue_size=5)

        self._signal_pub = rospy.Publisher('multibot_relay/signal', std_msgs.msg.String, queue_size=1)
        self._talking_pub = rospy.Publisher('multibot_relay/talking', std_msgs.msg.String, queue_size=1)

        self._signal_event = threading.Event()
        self._signal_waiting_id = ""
        self._signal_data = ""

        self._signal_waiting_list = []


    def _signal_callback(self, msg):

        if len(self._signal_waiting_list) > 0:
            for waiting_signal_info in list(self._signal_waiting_list):
                if waiting_signal_info['id'] == "" or msg.id == waiting_signal_info['id']:
                    if waiting_signal_info['signal'] == msg.signal:
                        #this is the signal we are waiting for
                        waiting_signal_info['event'].set()
                        #remove the signal object
                        self._signal_waiting_list.remove(waiting_signal_info)

    def _talking_callback(self, msg):
        pass

    def send_signal(self, signal):
        msg = std_msgs.msg.String()
        msg.data = signal
        rospy.loginfo('publishing {}'.format(signal))
        #publish at a 10Hz for 1 second
        rate = rospy.Rate(10)
        for x in range(0,10):
            self._signal_pub.publish(msg)
            rate.sleep()

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
    face.send_signal('HelloWorld')
    rospy.spin()

if __name__ == '__main__':
    main()
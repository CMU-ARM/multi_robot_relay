# tbd_tbd_multi_robot_relay

COPYRIGHT(C) 2019 - Transportation, Bots, and Disability Lab, Carnegie Mellon University  
Code released under MIT  
Contact - Zhi - zhi.tan@ri.cmu.edu  

A simple ROS application that passess information between two ROS system using roslib.js.

## How to use:
1. Clone this repo on both ROS systems.
1. Launch the relay by `roslauch tbd_multi_robot_relay start_relay.launch hostname:=XXXXX robot_id:=THIS_ROBOT_NAME` where `hostname` is the IP/hostname of the other machine. Launch the relay on both machines.

## Python Interface
It is highly recommende to use the python interface to communicate:

```
from tbd_multi_robot_relay import MultiBotInterface

interface = MultiBotInterface()
signal = interface.wait_for_robot(OTHER_ROBOT_ID) #wait for any signal from other robot
interface.send_signal('DONE') #send a text signal to all subscribing robots

```


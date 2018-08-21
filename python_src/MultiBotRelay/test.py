from __future__ import print_function
import roslibpy

ros = roslibpy.Ros(host='192.168.0.200', port=9090)
ros.on_ready(lambda: print('Is ROS connected?', ros.is_connected))
ros.run_forever()
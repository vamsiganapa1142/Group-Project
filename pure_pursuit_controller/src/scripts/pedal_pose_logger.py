#!/usr/bin/env python
import rospy
import numpy as np
import atexit
import tf
import os 
from os.path import expanduser
from time import gmtime, strftime
from numpy import linalg as LA
from motion_msgs.msg import ManualControl
import rospkg 



class PedalLogger():

    def __init__(self):

        # use the rospack object to get paths
        rospack = rospkg.RosPack()
        #get the path for this paackage
        package_path=rospack.get_path('pure_pursuit_controller')
        # get the pid to create "unique" filenames
        self.filename=package_path+'/waypoints/pedal_{}.csv'.format(os.getpid())
        self.file = open(self.filename, 'w')

    def save_pedal(self,data):
        print("x: {}, y: {}".format(data.Throttle,data.Steering))
        self.file.write('%f, %f\n' % (data.Throttle,data.Steering))

    def shutdown(self):
        self.file.close()
        print('Goodbye')
 
    def listener(self):
        rospy.init_node('Pedal_logger', anonymous=True)
        rospy.Subscriber('/pedal_pos', ManualControl, self.save_pedal)
        rospy.spin()

if __name__ == '__main__':

    # create Waypoint Object
    wp = PedalLogger()
    atexit.register(wp.shutdown)
    print('Saving pedal pos...')
    try:
        wp.listener()
    except rospy.ROSInterruptException:
        pass

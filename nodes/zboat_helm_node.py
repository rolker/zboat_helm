#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2018, All rights reserved.

import rospy
from geometry_msgs.msg import TwistStamped
import zboat_helm.zboat


class ZBoatHelm:
    def __init__(self):
        self.zboat = zboat_helm.zboat.ZBoat()
        
    def twistCallback(self,data):
        rospy.loginfo(rospy.get_caller_id() + ": ", data)
    
    def run(self):
        rospy.init_node('zboat_helm')
        rospy.Subscriber('cmd_vel',TwistStamped,self.twistCallback)
        rospy.spin()
        
if __name__ == '__main__':
    zbh = ZBoatHelm()
    zbh.run()
    

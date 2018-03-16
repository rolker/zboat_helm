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
        thrust = 1.5-data.twist.linear.x/2.0
        rudder = 1.5-data.twist.angular.z/2.0
        print thrust,rudder
        self.zboat.set_autonomy_mode()
        self.zboat.write_pwm_values(thrust, thrust, rudder)
    
    def run(self):
        self.zboat.open_port()
        self.zboat.set_autonomy_mode()
        rospy.init_node('zboat_helm')
        rospy.Subscriber('cmd_vel',TwistStamped,self.twistCallback)
        rospy.spin()
        
if __name__ == '__main__':
    zbh = ZBoatHelm()
    zbh.run()
    

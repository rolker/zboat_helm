#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2018, All rights reserved.

import rospy
from geometry_msgs.msg import TwistStamped
import zboat_helm.zboat
from marine_msgs.msg import Helm, Heartbeat, KeyValue

class ZBoatHelm:
    def __init__(self):
        self.zboat = zboat_helm.zboat.ZBoat()
        self.pilotingMode = 'standby'
        
    def twistCallback(self,data):
        thrust = 1.5-data.twist.linear.x/2.0
        rudder = 1.5-data.twist.angular.z/2.0
        print thrust,rudder
        self.zboat.set_autonomy_mode()
        self.zboat.write_pwm_values(thrust, thrust, rudder)
    
    def helmCallback(self,data):
        thrust = 1.5-data.throttle/2.0
        rudder = 1.5-data.rudder/2.0
        print thrust,rudder
        self.zboat.set_autonomy_mode()
        self.zboat.write_pwm_values(thrust, thrust, rudder)

    def pilotingModeCallback(self,data):
        self.pilotingMode = data.data

    def vehicleStatusCallback(self,event):
        hb = Heartbeat()
        nb.header.stamp = rospy.get_rostime()
        kv = KeyValue()
        kv.key = "piloting_mode"
        kv.value = self.pilotingMode
        hb.values.append(kv)
    
        self.heartbeat_pub.publish(hb)
        
    def run(self):
        self.zboat.open_port()
        self.zboat.set_autonomy_mode()
        rospy.init_node('zboat_helm')
        rospy.Subscriber('cmd_vel',TwistStamped,self.twistCallback)
        rospy.Subscriber('helm',Helm,self.helmCallback)
        rospy.Subscriber('/project11/piloting_mode', String, self.pilotingModeCallback)
        self.heartbeat_pub = rospy.Publisher('/heartbeat',Heartbeat,queue_size=1)
        rospy.spin()
        
if __name__ == '__main__':
    zbh = ZBoatHelm()
    zbh.run()
    

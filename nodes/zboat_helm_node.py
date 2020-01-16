#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2018, All rights reserved.

import rospy
from geometry_msgs.msg import TwistStamped
import zboat_helm.zboat
from marine_msgs.msg import Helm, Heartbeat, KeyValue, NavEulerStamped
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64

from dynamic_reconfigure.server import Server
from zboat_helm.cfg import zboat_helmConfig

class ZBoatHelm:
    def __init__(self):
        self.zboat = zboat_helm.zboat.ZBoat()
        self.pilotingMode = 'standby'
        self.speed_limiter = 0.15
        self.magnetic_declination = 0.0
        
    def twistCallback(self,data):
        self.applyThrustRudder(data.twist.linear.x,-data.twist.angular.z)
    
    def helmCallback(self,data):
        self.applyThrustRudder(data.throttle, data.rudder)

    def applyThrustRudder(self, thrust, rudder):
        t = 1.5-(self.speed_limiter*min(1.0,max(-1.0,thrust)))/2.0
        r = 1.5 #+rudder/2.0
        
        tl = t+(self.speed_limiter*min(1.0,max(-1.0,rudder)))/2.0
        tr = t-(self.speed_limiter*min(1.0,max(-1.0,rudder)))/2.0
        
        self.pwm_left_pub.publish(tl)
        self.pwm_right_pub.publish(tr)
        self.pwm_rudder_pub.publish(r)
        #self.zboat.set_autonomy_mode()
        self.zboat.write_pwm_values(tl, tr, r)

    def pilotingModeCallback(self,data):
        self.pilotingMode = data.data
        if data.data == 'standby':
            self.zboat.set_manual_mode()
        else:
            self.zboat.set_autonomy_mode()
        

    def vehicleStatusCallback(self,event):
        hb = Heartbeat()
        hb.header.stamp = rospy.get_rostime()
        kv = KeyValue()
        kv.key = "piloting_mode"
        kv.value = self.pilotingMode
        hb.values.append(kv)
    
        self.heartbeat_pub.publish(hb)
        
    def headingCallback(self,data):
        nes = NavEulerStamped()
        nes.header.stamp = rospy.get_rostime()
        nes.orientation.heading = data.data+self.magnetic_declination
        self.heading_pub.publish(nes)

    def reconfigure_callback(self, config, level):
        self.speed_limiter = config['speed_limiter']
        self.magnetic_declination = config['magnetic_declination']
        return config
        
    def run(self):
        self.zboat.open_port()
        self.zboat.set_autonomy_mode()
        rospy.init_node('zboat_helm')
        rospy.Subscriber('cmd_vel',TwistStamped,self.twistCallback)
        rospy.Subscriber('helm',Helm,self.helmCallback)
        rospy.Subscriber('/project11/piloting_mode', String, self.pilotingModeCallback)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.headingCallback)
        
        self.heartbeat_pub = rospy.Publisher('/heartbeat',Heartbeat,queue_size=1)
        self.pwm_left_pub = rospy.Publisher('/zboat/pwm/left',Float32,queue_size=1)
        self.pwm_right_pub = rospy.Publisher('/zboat/pwm/right',Float32,queue_size=1)
        self.pwm_rudder_pub = rospy.Publisher('/zboat/pwm/rudder',Float32,queue_size=1)
        self.heading_pub = rospy.Publisher('/heading', NavEulerStamped,queue_size=1)
        
        srv = Server(zboat_helmConfig, self.reconfigure_callback)
        
        rospy.Timer(rospy.Duration.from_sec(0.2),self.vehicleStatusCallback)
        rospy.spin()
        
if __name__ == '__main__':
    zbh = ZBoatHelm()
    zbh.run()
    

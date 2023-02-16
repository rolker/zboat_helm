#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2018, All rights reserved.

import sys
import rospy
from geometry_msgs.msg import TwistStamped
import zboat_helm.zboat
from project11_msgs.msg import Helm, Heartbeat, KeyValue, NavEulerStamped
from std_msgs.msg import String
from std_msgs.msg import Float32, Float64
import math 

from dynamic_reconfigure.server import Server
from zboat_helm.cfg import zboat_helmConfig

class ZBoatHelm:
    def __init__(self, serial_port):
        self.zboat = zboat_helm.zboat.ZBoat(serial_port)
        self.pilotingMode = 'standby'
        
        self.speed_limiter = 0.15
        self.turn_speed_limiter = 0.3
        self.differential_bias = 0.44
        
        self.magnetic_declination = 0.0
        
        self.heading = None
        
        self.helm_command = {}
        self.desired_command = {}
        
    def twistCallback(self,data):
        self.helm_command['throttle'] = data.twist.linear.x
        self.helm_command['rudder'] = -data.twist.angular.z
        self.helm_command['timestamp'] = data.header.stamp
        self.applyThrustRudder()
    
    def helmCallback(self,data):
        self.helm_command['throttle'] = data.throttle
        self.helm_command['rudder'] = data.rudder
        self.helm_command['timestamp'] = data.header.stamp
        self.applyThrustRudder()

    def desiredSpeedCallback(self, data):
        self.desired_command['speed'] = data.twist.linear.x
        self.desired_command['speed_timestamp'] = data.header.stamp

    def desiredHeadingCallback(self, data):
        self.desired_command['heading'] = data.orientation.heading
        self.desired_command['heading_timestamp'] = data.header.stamp
        self.applyThrustRudder()
        
    def applyThrustRudder(self):
        thrust = 0
        rudder = 0
        
        now = rospy.get_rostime()
        
        doDesired = True
        
        if 'timestamp' in self.helm_command:
            if (now - self.helm_command['timestamp']) < rospy.Duration.from_sec(0.5):
                doDesired = False
                
        if doDesired:
            if 'heading_timestamp' in self.desired_command and 'speed_timestamp' in self.desired_command and (now - self.desired_command['heading_timestamp']) < rospy.Duration.from_sec(0.5) and (now - self.desired_command['speed_timestamp']) < rospy.Duration.from_sec(0.5):
                delta_heading = self.desired_command['heading'] - self.heading
                while delta_heading > 180.0:
                    delta_heading -= 360.0
                while delta_heading < -180.0:
                    delta_heading += 360.0;
                rudder = max(-1,min(math.radians(delta_heading),1.0))
                thrust = max(-1,min(self.desired_command['speed'],1.0))
        else:
            thrust = self.helm_command['throttle']
            rudder = self.helm_command['rudder']
        
        #clamp values to range -1 to 1
        thrust = max(-1.0,min(1.0,thrust))
        rudder = max(-1.0,min(1.0,rudder))
        
        rudder_mag = abs(rudder)
        inv_rudder_mag = 1.0-rudder_mag
        
        #reduce thrust while turning
        thrust *= inv_rudder_mag
        
        #apply speed limiters
        thrust *= self.speed_limiter
        rudder *= self.turn_speed_limiter
        
        t = 1.5-(thrust/2.0)
        r = 1.5 #+rudder/2.0

        fwd_bias = self.differential_bias*2.0
        rev_bias = (1.0-self.differential_bias)*2.0
        
        if rudder > 0: #turning right
            tl = t-(fwd_bias*rudder/2.0)
            tr = t+(rev_bias*rudder/2.0)
        else:
            tl = t-(rev_bias*rudder/2.0)
            tr = t+(fwd_bias*rudder/2.0)
                    
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
        self.heading = data.data+self.magnetic_declination
        nes = NavEulerStamped()
        nes.header.stamp = rospy.get_rostime()
        nes.orientation.heading = self.heading
        self.heading_pub.publish(nes)

    def reconfigure_callback(self, config, level):
        self.speed_limiter = config['speed_limiter']
        self.turn_speed_limiter = config['turn_speed_limiter']
        self.differential_bias = config['differential_bias']
        self.magnetic_declination = config['magnetic_declination']
        return config
        
    def run(self):
        self.zboat.open_port()
        self.zboat.set_autonomy_mode()
        rospy.init_node('zboat_helm')

        self.heartbeat_pub = rospy.Publisher('/heartbeat',Heartbeat,queue_size=1)
        self.pwm_left_pub = rospy.Publisher('/zboat/pwm/left',Float32,queue_size=1)
        self.pwm_right_pub = rospy.Publisher('/zboat/pwm/right',Float32,queue_size=1)
        self.pwm_rudder_pub = rospy.Publisher('/zboat/pwm/rudder',Float32,queue_size=1)
        self.heading_pub = rospy.Publisher('/heading', NavEulerStamped,queue_size=1)
        
        rospy.Subscriber('cmd_vel',TwistStamped,self.twistCallback)
        rospy.Subscriber('helm',Helm,self.helmCallback)
        rospy.Subscriber('/project11/piloting_mode', String, self.pilotingModeCallback)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.headingCallback)
        
        rospy.Subscriber('/project11/desired_speed', TwistStamped, self.desiredSpeedCallback)
        rospy.Subscriber('/project11/desired_heading', NavEulerStamped, self.desiredHeadingCallback)
    
        srv = Server(zboat_helmConfig, self.reconfigure_callback)
        
        rospy.Timer(rospy.Duration.from_sec(0.2),self.vehicleStatusCallback)
        rospy.spin()
        
if __name__ == '__main__':
    if len(sys.argv) > 1:
        serial_port = sys.argv[1]
    else:
        serial_port = '/dev/ttyUSB0'
    print 'zboat serial port: ',serial_port
    zbh = ZBoatHelm(serial_port)
    zbh.run()
    

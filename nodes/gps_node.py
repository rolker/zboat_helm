#!/usr/bin/env python

# Roland Arsenault
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2018, All rights reserved.

import rospy
from geometry_msgs.msg import TwistStamped
from mission_plan.msg import NavEulerStamped
from geographic_msgs.msg import GeoPointStamped
import zboat_helm.gps


class GPSNode:
    def __init__(self):
        self.gps = zboat_helm.gps.GPS()
        
    def run(self):
        rospy.init_node('zboat_gps')
        sog_pub = rospy.Publisher('/sog', TwistStamped, queue_size=5)
        h_pub = rospy.Publisher('/heading', NavEulerStamped, queue_size=5)
        pos_pub = rospy.Publisher('/position', GeoPointStamped, queue_size=5)
        while not rospy.is_shutdown():
          line = self.gps.read()
          #print line
          if line and line.startswith('$GPVTG'):
              parts = line.split(',')
              #print parts
              if len(parts) >= 8:
                  ms = float(parts[7])*1000/3600.0
                  #print ms
                  ts = TwistStamped()
                  ts.twist.linear.x = ms
                  sog_pub.publish(ts)
          if line and line.startswith('$GPHDT'):
              parts = line.split(',')
              if len(parts) >= 1:
                  h = float(parts[1]) #*2*3.14/360.0
                  nes = NavEulerStamped()
                  nes.orientation.heading = h
                  h_pub.publish(nes)
          if line and line.startswith('$GPGGA'):
              parts = line.split(',')
              if len(parts) >= 6:
                lat = float(parts[2][:2])+float(parts[2][2:])/60.0
                if parts[3] == 'S':
                  lat = -lat
                lon = float(parts[4][:3])+float(parts[4][3:])/60.0
                if parts[5] == 'W':
                  lon = -lon
                gps = GeoPointStamped()
                gps.position.latitude = lat
                gps.position.longitude = lon
                pos_pub.publish(gps)
                 
        
if __name__ == '__main__':
    gn = GPSNode()
    gn.run()
    

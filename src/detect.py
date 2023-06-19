#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
import matplotlib.pyplot as plt
import math

#initialize global vars
pos = [0,0,0] #x,y,z
orient = [0,0,0] #roll, pitch, yaw
ranges = np.zeros(360)
intens = np.zeros(360)

spd = 0.1
obsDetected = False

def pclCb(data):
   global ranges
   global intens

   #set ranges and intensities from data
   ranges = np.array(data.ranges)
   intens = np.array([data.intensities])

#goes forward, stops when encounters object
def findObject(d):
   global obsDetected
   pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
   mov = Twist()
   if(ranges[0] < d and ranges[0] != 0):
      # mov.linear.x = 0
      print("Object found")
      obsDetected = True
   else:
      mov.linear.x = spd
      pub.publish(mov)


# takes list of orientation x, y, z, w quaternion values
# returns tuple in Euler form
def getEuler(quat):
   return(euler_from_quaternion(quat))


#simple code to test detection directly in front
#goes forward until it detects an object (w/n 0.5m), then stops
def main():
   rospy.init_node('detect', anonymous=True)
   rospy.Subscriber('scan',LaserScan, pclCb)

   r = rospy.Rate(10)

   while not rospy.is_shutdown() and not obsDetected:
      print("Range:", ranges[0])
      findObject(0.5)
      r.sleep()

if __name__ == "__main__":
   main()


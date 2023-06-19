#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np
import matplotlib.pyplot as plt
import math

#initialize global vars
pos = [0,0,0] #x,y,z
quat = [0,0,0,0] #x,y,z,w
orient = [0,0,0] #roll, pitch, yaw
ranges = np.zeros(360)
intens = np.zeros(360)

def odometryCb(data):
   global pos
   global orient
   global quat

   loc = data.pose.pose.position
   att = data.pose.pose.orientation

   #set position and orientation from data
   pos = [loc.x, loc.y, loc.z]
   quat = [att.x,att.y,att.z,att.w]
   orient = list(getEuler(np.array(quat)))
  
def pclCb(data):
   global ranges
   global intens

   #set ranges and intensities from data
   ranges = np.array(data.ranges)
   intens = np.array([data.intensities])

# takes list of orientation x, y, z, w quaternion values
# returns tuple in Euler form
def getEuler(quat):
   return(euler_from_quaternion(quat))

# use to print out odometry and laser information related
# to the turtlebot's current state
def main():
   rospy.init_node('odometry', anonymous=True)
   rospy.Subscriber('odom',Odometry, odometryCb)
   rospy.Subscriber('scan',LaserScan, pclCb)
   zeros = np.zeros(60)
   while not rospy.is_shutdown():
      print("Pos:", np.round(pos, 2)) # x, y, z position
      print("Orient:", np.round(orient, 2)) # roll, pitch, yaw
      # print("Angle:", np.round(orient[2], 2)) # orientation angle
      # print("Quat:", quat) # orientation in quaternion form
      # print("Ranges:", ranges)
      # print("Intensities:", intens)

if __name__ == "__main__":
   main()


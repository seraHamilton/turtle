#!/usr/bin/env python3
import rospy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from time import time


import numpy as np
import math

#define math functions
cos = math.cos
sin = math.sin
tanh = math.tanh
atan2 = math.atan2
sqrt = math.sqrt
pi = math.pi

#initialize global vars
pos = [0,0,0] #x,y,z position
orient = [0,0,0] #roll, pitch, yaw
ranges = [100] # list of len 360 providing distance to nearest obstacle
intens = np.zeros(360)
destReached = False # True when reaches goal

# vars to provide delay in turning
last = 0 # time of last turn
delay = 0.5

# variables to adjust scaling of velocity (ku) and angular velocity (kw)
ku = 1.1
kw = 0.8/pi

# define radii
# for small box w stickers
rhoi = 0.2 # radius of box
rho = 0.1 # radius of bot
rhoe = 0.2 # adjusts how close bot can get to box
rhoFi = rhoi + rhoe + 3*rho # sets distance at which bot is far enough from obstacle

#goal coordinates
xg = 1.5 # positive = forward
yg = 0 # negative = right

# initial offset of turtle from origin
cx = 0
cy = 0
ct = 0

def control():
   #x and y position at current time t
   xt = pos[0] - xg
   yt = pos[1] - yg

   theta = orient[2]

   #find x and y pos of obstacle
   xoi, yoi = setObs(xt, yt, theta)

   # find F goal
   Fgx = xt**2-yt**2
   Fgy = 2*xt*yt

   # find F obstacle
   Foxi, Foyi = Fo(xt, yt, xoi, yoi)

   # find F
   Fx, Fy = F(Fgx, Fgy, Foxi, Foyi, xt, yt, xoi, yoi)

   # find phi
   phi = atan2(Fy, Fx)

   # find velocity and angular velocity
   u = ku*tanh(xt**2 + yt**2)
   w = -kw*(theta-phi) # + phidot, ignore for now

   move(phi, u, w)

   # print current position
   print("current pos: (", pos[0], ",", pos[1], ")", sep='')
   
# sets location of obstacle
def setObs(xt, yt, theta):
   nzRng = [r for r in ranges if r != 0]

   # finds distance to nearest obstacle
   if len(nzRng) != 0:
      obsDist = min(nzRng)
      obsInd = ranges.index(obsDist) + rhoi
   else:
      obsDist = 100
      obsInd = 0

   if theta < 0:
      theta += 2*pi

   theta = theta*180/pi

   # angle at which obstacle exists
   obsAng = obsInd + theta

   # x and y position of obstacle
   xoi = xt + obsDist*cos(obsAng*pi/180)
   yoi = yt + obsDist*sin(obsAng*pi/180)

   return xoi, yoi

# returns tuple of Fx and y obstacle
def Fo(xt, yt, xoi, yoi):
   phii = atan2(-yoi, -xoi) + pi
   pxi = cos(phii)
   pyi = sin(phii)

   Foxi = pyi*(xt-xoi)*(yt-yoi) - pxi*(yt-yoi)**2
   Foyi = pxi*(xt-xoi)*(yt-yoi) - pyi*(xt-xoi)**2

   return Foxi, Foyi

# Determines F
def F(Fgx, Fgy, Foxi, Foyi, xt, yt, xoi, yoi):
   sigma = getSigma(xt, yt, xoi, yoi)

   Fx = sigma*Fgx+(1-sigma)*Foxi
   Fy = sigma*Fgy+(1-sigma)*Foyi

   return Fx, Fy

# find sigma based on distance to nearest obstacle
def getSigma(xt, yt, xoi, yoi):
   Bi = round(rhoi**2 - (xt-xoi)**2 - (yt - yoi)**2, 2)
   BiZ = round(-2*rhoi*(rho+rhoe) - (rho + rhoe)**2, 2)
   BiF = round(rhoi**2 - rhoFi**2, 2)

   if Bi <= BiF:
      sigma = 1
   elif Bi > BiF and Bi < BiZ:
      a = 2/(BiZ-BiF)**3
      b = -3*(BiZ+BiF)/(BiZ-BiF)**3
      c = 6*BiZ*BiF/(BiZ-BiF)**3
      d = (BiZ**2)*(BiZ-3*BiF)/(BiZ-BiF)**3
      sigma = a*Bi**3 + b*Bi**2 + c*Bi + d
   else:
      sigma = 0

   return sigma

def move(goal, u, w):
   global last
   pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)

   mov = Twist()
   if(abs(round(u, 2)) != 0):
      mov.linear.x = 0

      ang = orient[2]
      g = goal

      if ang < 0:
         ang += 2*pi

      if g < 0:
         g += 2*pi

      diff = g - ang

      # set turn direction
      if ang < pi:
         if diff < pi and diff > 0:
            mov.angular.z = 0.3
         else:
            mov.angular.z = -0.3
      else:
         if diff < -pi or (diff < pi and diff > 0):
            mov.angular.z = 0.3
         else:
            mov.angular.z = -0.3

      r = rospy.Rate(10) #delay 10 Hz

      # check if it has turned recently, if not, turn
      if time() > last + delay:
         while(abs(orient[2] - goal) >= 0.7) and not rospy.is_shutdown():
            pub.publish(mov)
            print("orient:", orient[2], "goal:", goal)
            r.sleep()
         last = time()

   #set linear and angular velocity
   mov.linear.x = round(u, 2)
   mov.angular.z = round(w, 2)
   print("u:", mov.linear.x, "w:", mov.angular.z)

   # if it has stopped moving, destination is reached
   global destReached
   destReached = mov.linear.x == 0 and mov.angular.z == 0

   pub.publish(mov)
   # mov.angular.z = 0

# records odometry data of bot
def odometryCb(data):
   global pos
   global orient

   # get position and orientation of bot
   # note: this is based on wheel rotations
   loc = data.pose.pose.position
   att = data.pose.pose.orientation # in quaternion form

   #set position and orientation from data
   pos = np.round(np.array([loc.x - cx, loc.y - cy, loc.z ]), 2) # [x, y, z]
   orient = getEuler(np.array([att.x, att.y, att.z, att.w])) #[roll, pitch, yaw]
   orient[2] = round(orient[2] - ct, 2) # set angle based on initiall offset

# records laser data of bot
def pclCb(data):
   global ranges
   global intens

   #set ranges and intensities from data
   ranges = data.ranges
   intens = np.array(data.intensities)

# takes np list of orientation x, y, z, w quaternion values
# returns list in Euler form
def getEuler(q):
   return np.array(euler_from_quaternion(q))

def main():
   begin = time()
   rospy.init_node('odometry', anonymous=True)

   # begin recording odometry and lidar data
   rospy.Subscriber('odom',Odometry, odometryCb)
   rospy.Subscriber('scan',LaserScan, pclCb)

   r = rospy.Rate(10) #delay 10 Hz

   global cx, cy, ct, xg, yg, destReached

   start = time()
   duration = 1

   # set x and y offset
   # must compare with 0 to give system time to load
   while(cx == 0 and cy == 0 and time() < start + duration):
      cx, cy = pos[0], pos[1]

   # set theta offset
   ct = orient[2]

   # runs controller until destination is reached
   # or until program is interuppted
   while not rospy.is_shutdown() and not destReached:
      control()
      r.sleep() # implements 10 Hz

   target = round(sqrt((pos[0] - xg)**2 + (pos[1] - yg)**2), 2)
   print("Exit with position (", pos[0], ",", pos[1], "), ", target, " away from target location of (", xg, ",", yg, ")", sep='')
   print("Time taken:", round(time() - begin, 2), "seconds")

if __name__ == "__main__":
   main()
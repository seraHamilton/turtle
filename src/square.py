#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from time import time

#moves turtle forward at set speed
def forward(spd):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)

    mov = Twist()
    mov.linear.x = spd

    start = time()
    duration = 5

    while time() < start+duration:
        pub.publish(mov)
    mov.linear.x = 0
    mov.angular.z = 0.4
    while time() < start + 2*duration:
        pub.publish(mov)

# moves turtlebot in square
# not based on odometry data
def main():
    rospy.init_node('forward_script',anonymous=True)

    count = 0
    speed = 0.22
    
    while count < 4:
        forward(speed)
        count += 1

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from time import time

#moves turtle forward at set speed
def forward(spd):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)

    mov = Twist()
    mov.linear.x = spd

    pub.publish(mov)

def main():
    rospy.init_node('forward_script',anonymous=True)

    # set cycle of 10 Hz to prevent turtlebot from moving
    # once the loop completes
    r = rospy.Rate(10)

    start = time()
    duration = 3

    speed = .22

    while time() < start+duration:
        forward(speed)
        r.sleep()
    print("done")

if __name__ == '__main__':
    main()
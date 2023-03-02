#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, Twist
import time

def main():
    rospy.init_node('track_circle')
    pub = rospy.Publisher("/locobot/mobile_base/goal_pose", Pose, queue_size=1)

    pub = rospy.Publisher("/locobot/mobile_base/commands/velocity", Twist, queue_size=1)
    for i in range(100):
        t = Twist()
        t.angular.z = 1.5
        pub.publish(t)
        print("publishing")
        time.sleep(0.1)
    return

    rospy.sleep(1)

    t_init = rospy.get_time()
    print(t_init)
    t = 0
    rate = rospy.Rate(50)
    while t <= 20:
        t = rospy.get_time() - t_init
        point = Pose()
        # point.position.x = .5 * np.cos(2*np.pi / 10 * t)
        # point.position.y = .5 * np.sin(2*np.pi / 10 * t)
        point.position.x = .1
        print("Publishing point:", point.position.x, point.position.y)
        pub.publish(point)
        rate.sleep()

if __name__ == '__main__':
    main()

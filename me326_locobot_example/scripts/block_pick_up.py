#!/usr/bin/env python3
from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script makes the end-effector draw a square in 3D space
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200'
# Then change to this directory and type 'python ee_cartesian_trajectory.py'
import numpy as np
import rospy
import tf2_ros
#from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs

class PickUp:

    def __init__(self) -> None:
        self.locobot = InterbotixLocobotXS("locobot_wx250s", "mobile_wx250s")
        #Starting configuration
        self.locobot.arm.go_to_sleep_pose()
        self.locobot.gripper.open()

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.arm_frame = 'locobot/arm_base_link'

        while True:
            try:
                self.transform = self.buffer.lookup_transform(self.arm_frame, 'locobot/camera_color_optical_frame', rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(.05)
        print("Got transform:", self.transform)

        # self.debug_pub = self.
    
    def move_arm(self, point: PointStamped):
        transformed_point = tf2_geometry_msgs.do_transform_point(point, self.transform)

        #Home position
        self.locobot.arm.go_to_home_pose()
        self.locobot.gripper.open()

        transformed_point.point.x += .01

        if transformed_point.point.z < -.1:
            print("Z coordinate is too low! limiting it")
            transformed_point.point.z = -.1
        
        #Move to block
        self.locobot.arm.set_ee_pose_components(x=transformed_point.point.x, y=transformed_point.point.y, z=.1, pitch=np.pi/2)
        self.locobot.arm.set_ee_pose_components(x=transformed_point.point.x, y=transformed_point.point.y, z=transformed_point.point.z - .01, pitch=np.pi/2)

        #close gripper
        # self.locobot.gripper.set_pressure(1.0)
        self.locobot.gripper.close(2.0)

        #return home
        self.locobot.arm.go_to_sleep_pose()
        rospy.sleep(3)

        #open gripper
        # self.locobot.gripper.open(2.0)


    def cube_pickup_callback(self, data: Marker):
        point = PointStamped()
        point.header = data.header
        point.point = data.pose.position
        self.move_arm(point)


def main():
    node = PickUp()
    rospy.Subscriber('/locobot/camera_cube_locator', Marker, node.cube_pickup_callback, queue_size=1)
    rospy.spin()

if __name__=='__main__':
    main()

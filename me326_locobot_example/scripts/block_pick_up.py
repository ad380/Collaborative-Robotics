from interbotix_xs_modules.locobot import InterbotixLocobotXS

# This script makes the end-effector draw a square in 3D space
#
# To get started, open a terminal and type...
# 'roslaunch interbotix_xslocobot_control xslocobot_python.launch robot_model:=locobot_wx200'
# Then change to this directory and type 'python ee_cartesian_trajectory.py'

import rospy
import tf2_ros
#from std_msgs.msg import String
from visualization_msgs.msg import Marker

def cube_pickup_callback(data: Marker):
    rospy.loginfo(rospy.get_caller_id() + "Position %s", data.pose.position.x)

    #initialize locobot 
    locobot = InterbotixLocobotXS("locobot_wx250s", "mobile_wx250s")
    #Starting configuration
    locobot.arm.go_to_sleep_pose()
    #Home position
    locobot.arm.go_to_home_pose()
    #Move to block
    locobot.arm.set_ee_cartesian_trajectory(z=-0.2)
    #locobot.arm.set_ee_cartesian_trajectory(x=-0.2)
    #locobot.arm.set_ee_cartesian_trajectory(z=0.2)
    #locobot.arm.set_ee_cartesian_trajectory(x=0.2)

    #close gripper
    locobot.gripper.set_pressure(1.0)
    locobot.gripper.close(2.0)

    #return home
    locobot.arm.go_to_sleep_pose()

    #open gripper
    locobot.gripper.open(2.0)

def main():
    rospy.Subscriber('/locobot/camera_cube_locator', Marker, cube_pickup_callback)
    rospy.spin()

if __name__=='__main__':
    main()

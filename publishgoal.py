#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point 
import roslaunch
import os


def movetogoal(xGoal,yGoal, oz, ow):
    #define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")
    goal = MoveBaseGoal()
    #set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/

    goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = oz
    goal.target_pose.pose.orientation.w = ow
    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    ac.wait_for_result(rospy.Duration(600))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")	
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
    return False


if __name__ == "__main__":
    rospy.init_node('map_navigation', anonymous=False)
    movetogoal(12.07, -0.95, -0.18, 0.984) 
    os.system('roslaunch kobuki_auto_docking activate.launch')

    '''
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/opt/ros/kinetic/share/kobuki_auto_docking/launch/activate.launch"])
    launch.start()
    rospy.sleep(300)
    '''

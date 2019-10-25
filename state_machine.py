#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import random
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from kobuki_msgs.msg import AutoDockingAction,AutoDockingGoal,PowerSystemEvent,SensorState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Twist

import actionlib
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import os
import numpy as np


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
    ac.wait_for_result(rospy.Duration(120))

    if(ac.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination")
    return False



# state machine:
# top-level states:
#  - charging
#  - undocking
#  - docking
#  - navigating
#
# transitions:
# - charged: charging->undocking
# - succeeded: undocking->navigating
# - succeeded: docking->charging
# - battery low: navigating->docking

class Charging(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['charged'])
        self.charged = False
        self.sub = rospy.Subscriber('/mobile_base/sensors/core', SensorState,
                        self.power_sub)

    def power_sub(self, msg):
        if msg.battery >= 150:
            rospy.loginfo("Charging done")
            self.charged = True
        else:
            self.charged = False

    def execute(self, userdata):
        #os.system('rosnode kill -a')
        #os.system('roslaunch turtlebot_bringup minimal.launch &') #only keep minimal nodes on
        # block here until we are charged

        ids = []
        ids.append(os.popen("ps -ef | grep move_base | head -1 | awk '{print $2}'").readlines()[0].strip())
        ids.append(os.popen("ps -ef | grep  __name:=camera_nodelet_manager | head -1 | awk '{print $2}'").readlines()[0].strip())
        print(ids)
        for item in ids:
            os.system('kill -STOP '+ item)


        while not self.charged and not rospy.is_shutdown():
            rospy.sleep(1)
            rospy.loginfo('waiting')

        rospy.loginfo('charged')
        # return charged
        for item in ids:
            os.system('kill -CONT '+ item)

        return 'charged'


class Undocking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.cmd_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist,
            queue_size=2)
        self.pose = Pose()

    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def execute(self, userdata):
        rospy.sleep(1)
        initial_pose = self.pose
        print('initial_pose', initial_pose)
        d = 0
        # back up for 0.5 meters to get away from dock
        while d < 1 and not rospy.is_shutdown():
            cmd = Twist()
            cmd.linear.x = -0.2
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
            pose = self.pose
            dx = self.pose.position.x - initial_pose.position.x
            dy = self.pose.position.y - initial_pose.position.y
            d = math.hypot(dx, dy)
            print(d, dx, dy)
            print(rospy.is_shutdown())
        # send a stop command for good measure
        cmd = Twist()
        self.cmd_pub.publish(cmd)
        return 'done'


class BatteryMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['low_battery', 'battery_ok'])
        self.low_battery = False
        self.sub = rospy.Subscriber('/mobile_base/sensors/core', SensorState,
                        self.power_sub)

    def power_sub(self, msg):
        if msg.battery <= 146:
            rospy.loginfo("Needs charging")
            self.low_battery = True
        else:
            rospy.loginfo("Battery OK")
            self.low_battery = False

    def execute(self, userdata):
        if self.low_battery:
            return 'low_battery'
        else:
            return 'battery_ok'


def timeout_command(command, timeout):
        """call shell-command and either return its output or kill it
        if it doesn't normally exit within timeout seconds and return None"""

        import subprocess, datetime, os, time, signal
        start = datetime.datetime.now()
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        while process.poll() is None:
            time.sleep(0.1)
            now = datetime.datetime.now()
            if (now - start).seconds> timeout:
                os.kill(process.pid, signal.SIGKILL)
                os.waitpid(-1, os.WNOHANG)
                return 'failed'
        return 'success'

class Docking(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['docked', 'failed'])
    def execute(self, userdata):
        movetogoal(12.07, -0.95, -0.18, 0.984)
        output = timeout_command(['roslaunch', 'kobuki_auto_docking', 'activate.launch'], 60)

        if output == 'failed':
            try:
                os.system('rosnode kill dock_drive_client_py')
            except:
                pass
            return 'failed'

        return 'docked'

class Navigating(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['continue', 'needs_charging'])
        self.charged = True
        self.sub = rospy.Subscriber('/mobile_base/sensors/core', SensorState,
                        self.power_sub)

        self.goals = [(15.53611591964232, -4.470042454101005, 0, 1), (7.873513100302757, 11.036855808341201, 0, 1) ]
    def power_sub(self, msg):
        if msg.battery <= 135:
            self.charged = False
        else:
            self.charged = True

    def execute(self, userdata):
        #os.system('rosnode kill -a')
        #os.system('roslaunch turtlebot_bringup minimal.launch &') #only keep minimal nodes on
        # block here until we are charged
        #while not self.charged and not rospy.is_shutdown():
        #    rospy.sleep(1)
        # return charged

        #os.system('roslaunch tr_autonav auto.launch &')
        goal = self.goals[np.random.randint(2)]
        movetogoal(*goal)
        #return 'charged'
        if not self.charged:
            return 'needs_charging'
        return 'continue'


rospy.init_node('tr_smach')
rospy.loginfo('nav demo started')
rospy.sleep(1)

sm = smach.StateMachine(['charged', 'done'])

with sm:
    smach.StateMachine.add('Undocking', Undocking(),
                           transitions={'done': 'Navigating'})
    smach.StateMachine.add('Docking', Docking(),
                           transitions={'docked': 'Charging', 'failed': 'Undocking'})
    smach.StateMachine.add('Charging', Charging(),
                           transitions={'charged': 'Undocking'})
    smach.StateMachine.add('Navigating', Navigating(),
                           transitions={'continue': 'Navigating', 'needs_charging': 'Docking'})

sm.execute()

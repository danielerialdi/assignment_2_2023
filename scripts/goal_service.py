#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import actionlib_msgs.msg
import actionlib_msgs
import assignment_2_2023.msg
from assignment_2_2023.msg import PlanningAction
from assignment_2_2023.msg import Info
from assignment_2_2023.srv import goalservice, goalserviceResponse
from tf import transformations
from std_srvs.srv import *
import time



def on_info(info):
    global goal_x, goal_y
    if(not 'goal_x' in globals()):
        goal_x = info.x
    if(not 'goal_y' in globals()):
        goal_y = info.y
        
        

def on_goal(goal_service):
    global goal_x, goal_y
    goal_x = goal_service.goal.target_pose.pose.position.x
    goal_y = goal_service.goal.target_pose.pose.position.y
    rospy.loginfo("Goal is %f, %f", goal_x, goal_y)
    
    
def on_service_call(s):
    global goal_x, goal_y
    return goalserviceResponse(goal_x, goal_y)

def main():
    rospy.init_node("goal_service")
    service = rospy.Service("goal_service", goalservice, on_service_call)
    sub_info = rospy.Subscriber("/info_pos_vel", Info, on_info)
    sub_goal = rospy.Subscriber("/reaching_goal/goal",assignment_2_2023.msg.PlanningActionGoal, on_goal)
    rospy.spin()


if __name__ == '__main__':
    main()

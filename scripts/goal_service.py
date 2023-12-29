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

def on_goal(goal_service):
    global position_x, position_y
    position_x = goal_service.goal.target_pose.pose.position.x
    position_y = goal_service.goal.target_pose.pose.position.y
    rospy.loginfo("Position is %d, %d", position_x, position_y)
    
    
def on_service_call(s):
    global position_x, position_y
    return goalserviceResponse(position_x, position_y)

def main():
    global position_x, position_y
    position_x = -100
    position_y = -100
    rospy.init_node("goal_service")
    service = rospy.Service("goal_service", goalservice, on_service_call)
    sub_goal = rospy.Subscriber("/reaching_goal/goal",assignment_2_2023.msg.PlanningActionGoal, on_goal)
    rospy.spin()


if __name__ == '__main__':
    main()

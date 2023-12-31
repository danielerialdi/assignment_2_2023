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
from assignment_2_2023.srv import infoservice, infoserviceResponse
from tf import transformations
from std_srvs.srv import *
import time
import math

def average(last):
    global window, window_index
    window[window_index] = last
    window_index = window_index + 1
    if(window_index == len(window)):
        window_index = 0
    sum_list = sum(window)
    avg = sum_list/len(window)
    return avg 



def on_goal(goal_service):
    global goal_x, goal_y
    goal_x = goal_service.goal.target_pose.pose.position.x
    goal_y = goal_service.goal.target_pose.pose.position.y
    #rospy.loginfo("Goal is %f, %f", goal_x, goal_y)


def on_info(info):
    global goal_x, goal_y, window_size, distance, average_velocity
    if(not 'goal_x' in globals()):
        goal_x = info.x
    if(not 'goal_y' in globals()):
        goal_y = info.y
    distance = math.sqrt((goal_x-info.x)**2 + (goal_y-info.y)**2)
    average_velocity = average(info.vel_linear_x)
    #rospy.loginfo("Distance is %f", distance)
    #rospy.loginfo("Average velocity is %f", average_velocity)
    #rospy.loginfo("Position is %f, %f, %f, %f", info.x, info.y, info.vel_linear_x, info.vel_angular_z)
    
    
    
def on_service_call(s):
    global distance, average_velocity
    return infoserviceResponse(distance, average_velocity)

def main():
    global window_size, window, window_index, distance, average_velocity
    rospy.init_node("info_service")
    if(rospy.has_param("/window_size")):
        window_size = rospy.get_param("/window_size")
    else:
        window_size = 20
    window = []
    window_index = 0
    for i in range(window_size):
        # Initialize the list of the size of my window_size
        window.append(0)
    distance = 0
    average_velocity = 0
    service = rospy.Service("info_service", infoservice, on_service_call)
    sub_info = rospy.Subscriber("/info_pos_vel", Info, on_info)
    sub_goal = rospy.Subscriber("/reaching_goal/goal",assignment_2_2023.msg.PlanningActionGoal, on_goal)
    rospy.spin()


if __name__ == '__main__':
    main()

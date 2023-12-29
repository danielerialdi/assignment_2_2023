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
from assignment_2_2023.srv import infoservice
from tf import transformations
from std_srvs.srv import *
import time

def on_info(info):
    #global position_x, position_y
    #position_x = goal_service.goal.target_pose.pose.position.x
    #position_y = goal_service.goal.target_pose.pose.position.y
    rospy.loginfo("Position is %f, %f, %f, %f", info.x, info.y, info.vel_linear_x, info.vel_angular_z)
    
    
    
def on_service_call(s):
    #global position_x, position_y
    return 

def main():
    global position_x, position_y
    position_x = -100
    position_y = -100
    rospy.init_node("info_service")
    service = rospy.Service("info_service", infoservice, on_service_call)
    sub_goal = rospy.Subscriber("/info_pos_vel", Info, on_info)
    rospy.spin()


if __name__ == '__main__':
    main()

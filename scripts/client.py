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
from actionlib_msgs.msg import GoalStatus
from tf import transformations
from std_srvs.srv import *
import time



def on_feedback(action_feedback):
    global canCancel
    canCancel = (action_feedback.status.status == action_feedback.status.ACTIVE)
    #rospy.loginfo("Feedback: pos_x = %f, pos_y = %f", action_feedback.feedback.actual_pose.position.x, action_feedback.feedback.actual_pose.position.y)
    rospy.loginfo("Feedback Status %d", action_feedback.status.status)
    if(action_feedback.status.status == action_feedback.status.SUCCEEDED):
        rospy.loginfo("Goal reached")
    elif(action_feedback.status.status == action_feedback.status.PREEMPTING):
        rospy.loginfo("Goal preempted")
        
def on_result(action_result):
    global canCancel
    canCancel = not(action_result.status.status == action_result.status.SUCCEEDED or action_result.status.status == action_result.status.PREEMPTED)
    rospy.loginfo("Result Status %d", action_result.status.status)


def pub_info(msg):
    global pub
    # global feedback
    # global hasGoal
    
    # feedback = assignment_2_2023.msg.PlanningFeedback()
    
    # Get the position
    pos = msg.pose.pose.position
	
	# Get the velocity
    vel_linear_x = msg.twist.twist.linear.x
    vel_angular_z = msg.twist.twist.angular.z
    rospy.loginfo("Velocity: %f, %f", vel_linear_x, vel_angular_z)
    
    # info message
    info = Info()
    info.x = pos.x
    info.y = pos.y
    info.vel_linear_x = vel_linear_x
    info.vel_angular_z = vel_angular_z
    pub.publish(info)


def main():
    global pub
    global canCancel
    canCancel = True
    rospy.init_node('handle_parameter_client')
    
    pub = rospy.Publisher('/info_pos_vel', Info, queue_size = 1)
    
    # Subscriber
    sub = rospy.Subscriber("/odom",Odometry,pub_info)
    
    # SUbscriber for feedback/status
    sub_feedback = rospy.Subscriber("/reaching_goal/feedback",assignment_2_2023.msg.PlanningActionFeedback, on_feedback)
    
    
    sub_result = rospy.Subscriber("/reaching_goal/result",assignment_2_2023.msg.PlanningActionResult, on_result)
    
    
    feedback = assignment_2_2023.msg.PlanningFeedback()
    
    goal = assignment_2_2023.msg.PlanningGoal()
    # Create the action client
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    
    # Wait for the server to be started
    client.wait_for_server()

    while not rospy.is_shutdown():
	  
        # Get the coordinates x and y from the user by keyboard
        x_position = float(input("Position X: "))
        y_position = float(input("Position Y: "))
        
        goal.target_pose.pose.position.x = x_position
        goal.target_pose.pose.position.y = y_position
        # Send the goal to the server
        client.send_goal(goal)
        rospy.sleep(0.3)
        print(canCancel)
        if(canCancel):
            print("Enter 'y' if you want to cancel the goal just selected")
            delete_char = input("Cancel? ")
            if(delete_char == 'y' and canCancel):
                client.cancel_goal()
            else:
                print("The goal has already been reached")
            
        
    rospy.spin()
    


if __name__ == '__main__':
    main()
    
    

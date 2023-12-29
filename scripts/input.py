#! /usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import Point, Pose, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import actionlib
import actionlib.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import SetParametersAction
from assignment_2_2023.msg import PlanningAction
from assignment_2_2023.msg import Info
from tf import transformations
from std_srvs.srv import *
import time

def trigger_response(request):
    return PlanningActionResponse(success = True, message = "Goal reached")	
	


def pub_info(msg):
    global pub
    #global feedback
    global hasGoal
    
    feedback = assignment_2_2023.msg.PlanningFeedback()
    
    # Get the position
    pos = msg.pose.pose.position
	
	# Get the velocity
    vel = msg.twist.twist.linear
    
    #rospy.loginfo("Vel = %d, %d", vel.x, vel.y)
    goal = assignment_2_2023.msg.PlanningGoal()
    if(hasGoal):
        # Setting feedback
        feedback.actual_pose.position.x = pos.x
        feedback.actual_pose.position.y = pos.y
        server.publish_feedback(feedback)
        rospy.loginfo("Feedback: pos_x = %f, pos_y = %f Vel = %d, %d", feedback.actual_pose.position.x, feedback.actual_pose.position.y , vel.x, vel.y)
    
    
    
    # info message
    info = Info()
    info.x = pos.x
    info.y = pos.y
    info.vel_x = vel.x
    info.vel_y = vel.y
    pub.publish(info)

def handle_parameter_server(goal):
    global server
    global hasGoal
    
    
    # Extract the desired position from the action goal
    desired_x = goal.target_pose.pose.position.x
    desired_y = goal.target_pose.pose.position.y
    
    
    result = assignment_2_2023.msg.PlanningResult()

    # Perform any actions or computations based on the received goal
    rospy.loginfo("Received goal: des_pos_x = %f, des_pos_y = %f", desired_x, desired_y)
    # goal.set_succeeded(desired_x, desired_y)
    hasGoal = True
    rospy.set_param('des_pos_x', desired_x)
    rospy.set_param('des_pos_y', desired_y)
    
    
    server.start()
    
    i = 0
    while i < 100:
        feedback = assignment_2_2023.msg.PlanningFeedback()
        
        time.sleep(1)
        
    server.set_succeeded(result, "msg")
        
    return result

'''

def client():
	# Create the action client
    client = actionlib.SimpleActionClient('server', SetParametersAction)
    
    # Wait for the server to be started
    client.wait_for_server()

    while not rospy.is_shutdown():
	  
      # Get the coordinates x and y from the user by keyboard
      x_position = float(input("Position X: "))
      y_position = float(input("Position Y: "))    
'''
    
def main():
    global hasGoal
    #global feedback
    hasGoal = False
    #feedback = assignment_2_2023.msg.PlanningFeedback()
    #result = assignment_2_2023.msg.PlanningResult()
    
    # Global variable pub 
    global pub
    global server
    # Initializing the publisher that will be used
    pub = rospy.Publisher('/info_pos_vel', Info, queue_size = 1)
    # Initialize the ROS node
    rospy.init_node('handle_parameter_server')

    #rospy.service('server_name', SetParametersAction, trigger_response)
    # Create an action server
    server = actionlib.SimpleActionServer('server', PlanningAction, handle_parameter_server, auto_start=False)
    server.start()
    
    # Subscriber
    sub = rospy.Subscriber("/odom",Odometry,pub_info)

    # Print a message indicating the server is ready
    rospy.loginfo("Handle Parameter Action Server is ready.")

    # Keep the script running
    rospy.spin()
    


if __name__ == '__main__':
    main()
    
    
    '''
    
def get_user_input():
    try:
        # Get user input for x and y coordinates
        x = float(raw_input("Enter desired x coordinate: "))
        y = float(raw_input("Enter desired y coordinate: "))
        return x, y
    except ValueError:
        rospy.logerr("Invalid input. Please enter valid numerical values.")
        return None, None

def handle_parameter_client():
    # Initialize the ROS node
    rospy.init_node('handle_parameter_client')

    # Create an action client
    client = actionlib.SimpleActionClient('handle_parameter', HandleParameterAction)

    # Wait for the action server to start
    client.wait_for_server()

    while not rospy.is_shutdown():
        # Get user input
        x, y = get_user_input()

        if x is not None and y is not None:
            # Create a goal
            goal = HandleParameterGoal()
            goal.desired_x = x
            goal.desired_y = y

            # Send the goal to the action server
            client.send_goal(goal)

            # Wait for the action to finish
            client.wait_for_result()

            # Get the result
            result = client.get_result()

            # Print the result
            rospy.loginfo("Action completed with result: %s", result)

            # Ask the user if they want to input another goal
            another_goal = raw_input("Do you want to input another goal? (y/n): ")
            if another_goal.lower() != 'y':
                break

if __name__ == '__main__':
    handle_parameter_client()
    
    '''

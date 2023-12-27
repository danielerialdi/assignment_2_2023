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
from tf import transformations
from std_srvs.srv import *
import time




def main():
    rospy.init_node('handle_parameter_client')
    goal = assignment_2_2023.msg.SetParametersGoal 
    # Create the action client
    client = actionlib.SimpleActionClient('server', SetParametersAction)
    
    # Wait for the server to be started
    client.wait_for_server()

    while not rospy.is_shutdown():
	  
        # Get the coordinates x and y from the user by keyboard
        x_position = float(input("Position X: "))
        y_position = float(input("Position Y: "))
        goal.desired_x = x_position
        goal.desired_y = y_position
        # Send the goal to the server
        client.send_goal(goal)
        
    rospy.spin()
    


if __name__ == '__main__':
    main()
    
    

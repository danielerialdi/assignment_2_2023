#! /usr/bin/env python

"""
.. module:: goal_service

    :platform: Unix
    :synopsis: Python module for assignment_2_2023 that implements a service to let
               the user know where the goal has been set.
    :description: In this module, a ROS node providing a service is implemented.
                  The goal is set by the user in a new terminal by typing the x and y coordinates.
                  The service is called by the user in a new terminal and it returns the last goal set by the user.
                  
                  Service:
                      - `goal_service`
                  
                  Subscriber:
                      - `/reaching_goal/goal`

.. moduleauthor:: Daniele Rialdi daniele.rialdi@gmail.com

"""
    
import rospy
import actionlib
import actionlib.msg
import actionlib_msgs.msg
import actionlib_msgs
import assignment_2_2023.msg
from assignment_2_2023.msg import PlanningAction
from assignment_2_2023.srv import goalservice, goalserviceResponse
# from assignment_2_2023.msg import Info


# Alternative way to init goal_x and goal_y using info
'''
def on_info(info):
    global goal_x, goal_y
    
    # It checks weather the goal is defined or not; if not, it stores in the goal_x and in goal_y the current position
    # This is done in the case the service is called before any goal is set by the user
    if(not 'goal_x' in globals()):
        goal_x = info.x
    if(not 'goal_y' in globals()):
        goal_y = info.y
        
'''        

def on_goal(goal_service):
    """ 
    Callback for the subscriber to /reaching_goal/goal.
    It stores the last goal set by the user.

    :param goal_service: message that contains the goal set by the user
    :type goal_service: assignment_2_2023.msg.PlanningActionGoal
    :param goal_x: x coordinate of the goal
    :type goal_x: float
    :param goal_y: y coordinate of the goal
    :type goal_y: float
    """
    global goal_x, goal_y
    goal_x = goal_service.goal.target_pose.pose.position.x
    goal_y = goal_service.goal.target_pose.pose.position.y
    rospy.loginfo("Goal is %f, %f", goal_x, goal_y)
    
    
def on_service_call(s):
    """ 
    Callback of the service, that let the user know where the goal has been set.

    :param s: service request
    :type s: assignment_2_2023.srv.goalservice
    :return: the last goal set by the user
    :rtype: assignment_2_2023.srv.goalserviceResponse
    """
    global goal_x, goal_y
    return goalserviceResponse(goal_x, goal_y)

def main():
    """
    Main function that initializes the node and the service.
    """
    global goal_x, goal_y
    rospy.init_node("goal_service_node")
    
    # This init is used when the service is called before any goal is set by the user
    goal_x = rospy.get_param('des_pos_x')
    goal_y = rospy.get_param('des_pos_y')
    
    # Service used to communicate the last target set by the user
    service = rospy.Service("goal_service", goalservice, on_service_call)

    # Subscribes /reaching_goal/goal to take the last goal coordinates set by the user
    sub_goal = rospy.Subscriber("/reaching_goal/goal", assignment_2_2023.msg.PlanningActionGoal, on_goal)
    
    
    # See comments at the top of the file, alternative init
    # Subscribes the info message published by the client
    # sub_info = rospy.Subscriber("/info_pos_vel", Info, on_info)
    
    rospy.spin()


if __name__ == '__main__':
    main()

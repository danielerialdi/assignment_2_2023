#! /usr/bin/env python

"""
.. module:: info_service
   :noindex:
   
   :platform: Unix
   :synopsis: Python module that implements a service to let the user know the distance of the robot from the target and the average linear velocity computed in a certain window.
   :description: In this module, ROS node providing a service is implemented.
                 The service is called by the user and it returns the distance of the robot from the target and the average linear velocity computed in a certain window.
                 The distance is computed using the Euclidean distance formula, while the average linear velocity is computed using a moving window of a certain size.
                 The size of the window is given by the parameter **window_size**.

                 Service:
                    - `info_service`

                 Subscribers:
                    - `/info_pos_vel`
                    - `/reaching_goal/goal`

.. moduleauthor:: Daniele Rialdi daniele.rialdi@gmail.com


"""

import rospy
import math
import actionlib
import actionlib.msg
import actionlib_msgs.msg
import actionlib_msgs
import assignment_2_2023.msg
from assignment_2_2023.msg import Info
from assignment_2_2023.srv import infoservice, infoserviceResponse


def average(last):
    """
    Function that computes the average of the last **window_size** elements.

    :param last: last element to be added to the window
    :type last: float
    :param window: list that contains the last **window_size** elements
    :type window: list
    :param window_index: index of the last element added to the window
    :type window_index: int
    :param sum_list: sum of the elements in the window
    :type sum_list: float
    :param avg: average of the elements in the window
    :type avg: float
    :return: average of the elements in the window
    :rtype: float
    """
    global window, window_index
    window[window_index] = last
    
    # Circular buffer
    window_index = window_index + 1
    if(window_index == len(window)):
        window_index = 0
    sum_list = sum(window)
    avg = sum_list/len(window)
    
    return avg 


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
    # rospy.loginfo("Goal is %f, %f", goal_x, goal_y)


def on_info(info):
    """
    Callback for the subscriber to /info_pos_vel. It computes the distance from the target of the robot and the average velocity computed in a certain window of the robot, given by the parameter **window_size**.

    :param info: message that contains the position and the velocity of the robot
    :type info: assignment_2_2023.msg.Info
    :param goal_x: x coordinate of the goal
    :type goal_x: float
    :param goal_y: y coordinate of the goal
    :type goal_y: float
    :param window_size: size of the window used to compute the average velocity
    :type window_size: int
    :param distance: distance of the robot from the target
    :type distance: float
    :param average_velocity: average linear velocity computed in a certain window
    :type average_velocity: float
    """
    global goal_x, goal_y, window_size, distance, average_velocity
    
    distance = math.sqrt((goal_x-info.x)**2 + (goal_y-info.y)**2)
    average_velocity = average(info.vel_linear_x)
    # rospy.loginfo("Distance is %f", distance)
    # rospy.loginfo("Average velocity is %f", average_velocity)
    # rospy.loginfo("Position is %f, %f, %f, %f", info.x, info.y, info.vel_linear_x, info.vel_angular_z)
    
    
    
def on_service_call(s):
    """
    Callback of the service, that let the user know the distance of the robot from the target and the average linear velocity computed in a certain window.
    
    :param s: service request
    :type s: assignment_2_2023.srv.infoservice
    :return: the distance of the robot from the target and the average linear velocity computed in a certain window
    :rtype: assignment_2_2023.srv.infoserviceResponse
    """
    global distance, average_velocity
    return infoserviceResponse(distance, average_velocity)

def main():
    """
    Main function that initializes the node and the service.
    """
    global window_size, window, window_index, distance, average_velocity, goal_x, goal_y
    rospy.init_node("info_service_node")
    
    # Initialize the moving window storage to compute average_velocity
    if(rospy.has_param("window_size")):
        window_size = rospy.get_param("window_size")
    else:
        window_size = 20
    window = []
    window_index = 0
    for i in range(window_size):
        # Initialize the list of the size of my window_size
        window.append(0)
    
    # Initialize distance and average_velocity
    distance = 0
    average_velocity = 0
    
    # This init is used when the service is called before any goal is set by the user
    goal_x = rospy.get_param('des_pos_x')
    goal_y = rospy.get_param('des_pos_y')
    
    # Service used to communicate the distance of the robot from the target and the average linear velocity
    # computed in a certain window
    service = rospy.Service("info_service", infoservice, on_service_call)
    
    # Subscribes the info message published by the client
    sub_info = rospy.Subscriber("/info_pos_vel", Info, on_info)
    
    # Subscribes /reaching_goal/goal to take the last goal coordinates set by the user
    sub_goal = rospy.Subscriber("/reaching_goal/goal",assignment_2_2023.msg.PlanningActionGoal, on_goal)
    
    rospy.spin()


if __name__ == '__main__':
    main()

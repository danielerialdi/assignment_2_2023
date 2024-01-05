#! /usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import actionlib
import actionlib.msg
import actionlib_msgs.msg
import assignment_2_2023.msg
from assignment_2_2023.msg import PlanningAction
from assignment_2_2023.msg import Info



def on_feedback(action_feedback):
    global canCancel
    
    # We can preempt the robot only if the state machine is in the "ACTIVE" status
    canCancel = (action_feedback.status.status == action_feedback.status.ACTIVE)
    
    #rospy.loginfo("Feedback: pos_x = %f, pos_y = %f", action_feedback.feedback.actual_pose.position.x, action_feedback.feedback.actual_pose.position.y)
    
    # Log to see when the robot reaches the goal or when it gets preempted
    #  rospy.loginfo("Feedback Status %d", action_feedback.status.status)
    # if(action_feedback.status.status == action_feedback.status.SUCCEEDED):
    #     rospy.loginfo("Goal reached")
    # elif(action_feedback.status.status == action_feedback.status.PREEMPTING):
    #     rospy.loginfo("Goal preempted")

        
def on_result(action_result):
    global canCancel
    # We can preempt the robot only if the state machine is not in the "SUCCEEDED" or in the "PREEMPTED" status
    canCancel = not(action_result.status.status == action_result.status.SUCCEEDED or action_result.status.status == action_result.status.PREEMPTED)
    
    #rospy.loginfo("Result Status %d", action_result.status.status)


def on_odom(msg):
    global pub_info
    
    # Get the current position
    pos = msg.pose.pose.position
	
	# Get the current velocity
    vel_linear_x = msg.twist.twist.linear.x
    vel_angular_z = msg.twist.twist.angular.z
    
    # rospy.loginfo("Velocity: %f, %f", vel_linear_x, vel_angular_z)
    
    # Prepare and publish the info message
    info = Info()
    info.x = pos.x
    info.y = pos.y
    info.vel_linear_x = vel_linear_x
    info.vel_angular_z = vel_angular_z
    
    pub_info.publish(info)


def main():
    # Declarations of global variables 
    global pub_info, canCancel
    canCancel = True
    rospy.init_node('client_node')
    
    # Declaring the publishers and subscribers
    # info_pub is a publisher that sends the info (x,y,vel_linear_x, vel_angular_z)
    pub_info = rospy.Publisher('/info_pos_vel', Info, queue_size = 1)
    
    # sub_odom is a subscriber that receives Odometry data
    sub_odom = rospy.Subscriber("/odom", Odometry, on_odom)
    
    # Subscriber for feedback/status
    sub_feedback = rospy.Subscriber("/reaching_goal/feedback",assignment_2_2023.msg.PlanningActionFeedback, on_feedback)
    
    # Subscriber for result
    sub_result = rospy.Subscriber("/reaching_goal/result",assignment_2_2023.msg.PlanningActionResult, on_result)
    
    # Creating an instance of PlanningGoal
    goal = assignment_2_2023.msg.PlanningGoal()
    
    # Create the action client that connects to the action server /reaching_goal
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    
    # Wait for the server to be started
    client.wait_for_server()

    while not rospy.is_shutdown():
    
        # Get the coordinates x and y from the user by keyboard checking that the input is valid
        while(True):
            x_position = input("Target X: ")
            try:
                x_position = float(x_position)
                break
            except:
                print("The input is not acceptable, please digit a number")
        while(True):
            y_position = input("Target Y: ")
            try:
                y_position = float(y_position)
                break
            except:
                print("The input is not acceptable, please digit a number")
        # Set and send the goal to the server
        goal.target_pose.pose.position.x = x_position
        goal.target_pose.pose.position.y = y_position
        client.send_goal(goal)
        
        # I have to wait a bit to ensure that the status is correctly set; if this is not done canCancel value 
        # would not be updated. I have to wait that the feedback is provided at least once.
        # With time = 0.1 it works on my machine, but to be sure a time = 0.3 is set precautionsly 
        rospy.sleep(0.3)
        
        if(canCancel):
            print("Enter 'y' if you want to cancel the goal just selected")
            while(canCancel):
                delete_char = input("Cancel? ").lower()
                if(delete_char == 'y'):
                    if(canCancel):
                        client.cancel_goal()
                        break
                    else:
                        print("The goal has already been reached")
                else:
                    print("The input is not valid")
        else:
            print("The goal has already been reached")

    rospy.spin()
    


if __name__ == '__main__':
    main()
    
    

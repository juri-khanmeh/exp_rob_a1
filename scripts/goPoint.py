## @package exp_rob_a1
# \file goPoint.py
# \brief This file contains the node for controlling the motion of robot in the package exp_rob_a1
# \author Juri Khanmeh
# \version 0.1
# \date 20/11/2021
#
# \details
#
# Subscribes to: <BR>
# ° /odom
#
# Publishes to: <BR>
# ° /cmd_vel
#
# Service : <BR>
# ° /go_point
#
# Description :
#
# This node controls the motion of the robot. 
# In order to reach a position, the controller sets the velocities
# depending on the error of current position and the target position
#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from exp_rob_a1.srv import Position
import math

# robot state variables
p_vel = Point()
vel = Twist()
pub_ = None

##
# \brief '/odom' subscribtion callback.
# \param msg an odometry variable which contains the position and the orientation.
# \return the current position_ of the robot.
#
# This function return a position
#
def clbk_odom(msg):
    global p_vel
    p_vel.x = msg.pose.pose.position.x
    p_vel.y = msg.pose.pose.position.y

##
# \brief motion control function
# \param goal is the target position that the robot must reach.
# \return feedback stats.
#
# This function changes the robot state according to 
# the current position of the robot and the target position.
# The objective of this function is to make the robot 
# reach a specific point.
#       
def go_to_point(req):
    T_vel = Point()
    T_vel.x = req.pos_x
    T_vel.y = req.pos_y
    k = 2
    
    while True:
    	if (math.fabs(T_vel.x - p_vel.x) < 0.1 and math.fabs(T_vel.y - p_vel.y) < 0.1) :
    	    vel.linear.x = 0
    	    vel.linear.y = 0
    	    pub_.publish(vel)
    	    break
    	else:
    	    vel.linear.x = k*(T_vel.x - p_vel.x)
    	    vel.linear.y = k*(T_vel.y - p_vel.y)
    	    pub_.publish(vel)
    		
    return True

##
# \brief main function
# \param null
# \return null
#
# This is the main function of go_point node
# It initializes the node, creates a '/cmd_vel' publisher, '/odom' subscriber,
# '/go_to_point' service server.
#
def main():
    global pub_
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    service = rospy.Service('/go_point', Position, go_to_point)
    rospy.spin()

if __name__ == '__main__':
    main()

#!/usr/bin/env python


#This node drives the robot based on information from the other nodes.

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32

import random
import math
import time

#Makes the state message global
def cb_state(msg):
    global state
    state = msg.data

#Makes the twist object sent from PID global
def cb_twist(msg):
    global t_pid
    t_pid = msg


def find_wall():
    global last_vel
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z= 0.0

    last_vel[0] = msg.linear.x
    last_vel[1] = msg.angular.z
    return msg

def follow_wall():
    global t_pid
    return t_pid

def turn_left():
    msg = Twist()
    msg.angular.z = 1
    msg.linear.x = 0
    return msg

#Init node
rospy.init_node('driver')

#Make publisher for cmd_vel
pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#Make all subscribers
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_pid_twist = rospy.Subscriber('pid_twist', Twist, cb_twist)

#Rate object
rate = rospy.Rate(10)

state = WONDERING


#Create two twist variable, one is modified here, one is copied from the PID messages
t_pid = Twist()


## global variables
last_vel = [random.uniform(0.1,0.3),  random.uniform(-0.3,0.3)]

print("STARTING")

while not rospy.is_shutdown():
    print("STATE: ", state)

    t_pub = Twist()

    if (state == WONDERING):
        t_pub = find_wall()

    if (state == FOLLOWING):
        #Calculate and set appropriate t_pub values
        t_pub = follow_wall()

    elif (state == TURNING):
        #Calculate and set appropriate t_pub values
        t_pub = turn_left()

    else:
        print("STATE NOT FOUND")
    pub_vel.publish(t_pub)
    rate.sleep()



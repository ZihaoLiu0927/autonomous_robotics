#!/usr/bin/env python

#This processes all of the scan values


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from state_definitions import *
from geometry_msgs.msg import Twist
import math


#global varibales
regions_ = {
    'fright': 0,
    'right': 0,
    'fleft': 0,
    'front': 0,
    'left': 0,
}

state_ = WONDERING

MAX_DIST = 1

# wall distance
wd = 0.2

LINEAR_SPEED = 0.3

e = 0

angle_min = 0

dist_front = 0

diff_e = 0

dist_min = 0

p = 1

d = 0.1

angle = 1 

t_pid_ = Twist()

def change_pid_component(msg):
    global wd, LINEAR_SPEED, e, diff_e, angle_min, dist_front, dist_min, t_pid_
    global p, d, angle

    size = len(msg.ranges)
    min_index = 0
    max_index = 126

    for i in range(min_index, max_index):
        if msg.ranges[i] < msg.ranges[min_index] and msg.ranges[i] > 0.12:
            min_index = i
    angle_min = min_index * msg.angle_increment
    dist_min = msg.ranges[min_index]
    dist_front = msg.ranges[0]
    diff_e = min((dist_min - wd) - e, 100)
    e = min(dist_min - wd, 100)

    t_pid_.angular.z = max(min((p*e+d*diff_e) + angle*(angle_min-((math.pi)/2)), 2.5), -2.5)
    
    if dist_front < wd:
        t_pid_.linear.x = 0
    elif dist_front < wd*2:
        t_pid_.linear.x = 0.5*LINEAR_SPEED
    elif abs(angle_min) > 1.75:
        t_pid_.linear.x = 0.4*LINEAR_SPEED
    else:
        t_pid_.linear.x = LINEAR_SPEED  

#Process all the data from the LIDAR
def cb(msg):
    #Determine state
    global regions_, state_, t_pid_
    regions_ = {
        'left':  min(min(msg.ranges[54:90]), MAX_DIST),
        'fleft': min(min(msg.ranges[18:54]), MAX_DIST),
        'front':  min(min(msg.ranges[342:359] + msg.ranges[0:18]), MAX_DIST),
        'fright':  min(min(msg.ranges[306:342]), MAX_DIST),
        'right':   min(min(msg.ranges[270:306]), MAX_DIST),
        'bleft': min(min(msg.ranges[90:126]), MAX_DIST),
    }

    change_pid_component(msg)
    identify_state()

    pub_pid.publish(t_pid_)
    pub_state.publish(state_)

def change_state(state):
    global state_
    state_ = state

def identify_state():
    global regions_, wd, state_
    regions = regions_
    
    state_description = ''
    
    # no obstacle
    if state_ != FOLLOWING and state_ != TURNING and regions['front'] == MAX_DIST and regions['fleft'] == MAX_DIST and regions['fright'] == MAX_DIST and regions['left'] == MAX_DIST and regions['right'] == MAX_DIST and regions['bleft'] == MAX_DIST:
        change_state(WONDERING)
    # obstacle right
    elif regions['front'] >= 2 * regions['bleft'] and regions['fleft'] >= 2 * regions['bleft']:
        change_state(TURNING)
    else :
        change_state(FOLLOWING)

#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
#THINK OF WHAT INFO TO PUBLISH TO THE PID
pub_pid = rospy.Publisher('pid_twist', Twist, queue_size = 1)

#Rate object
rate = rospy.Rate(10)


#Keep the node running
while not rospy.is_shutdown():
    print("working...")
    rate.sleep() 







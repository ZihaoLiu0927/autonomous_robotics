#!/usr/bin/env python

import rospy
import sys
import math
import tf
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion


# fill in scan callback
def scan_cb(msg):
    global state; global last_state
    for i in range(8):
        dist = msg.ranges[i * 45]
        if dist < 0.2 :
            state = "H"
            break

# it is not necessary to add more code here but it could be useful
def key_cb(msg):
    global state; global last_key_press_time; global radius;
    global last_state;
    if msg.data == "S":
        if state == "S":
            return
        else:
            radius = 0
    state = msg.data
    last_key_press_time = rospy.Time.now()

# odom is also not necessary but very useful
def odom_cb(msg):
    global curr_posi
    global theta
    curr_posi = msg.pose.pose.position

    new_angular = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([new_angular.x, new_angular.y, new_angular.z, new_angular.w])
    print(f'speed [linear, angular]: [{roll}, {yaw}]; location ({curr_posi.x}, {curr_posi.y}, {curr_posi.z})')
    return

# print the state of the robot
def print_state():
    print("---")
    print("STATE: " + state)

    # calculate time since last key stroke
    time_since = rospy.Time.now() - last_key_press_time
    print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

# init node
rospy.init_node('dancer')

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# 
LINEAR_SPEED = 0.2
ANGULAR_SPEED = math.pi/12
dic_velocity_vector = {
    "H" : [0, 0],
    "L" : [0, 1],
    "R" : [0, -1],
    "F" : [1, 0],
    "B" : [-1, 0],
    "S" : [1, 1],
    "Z" : [1, 2],
}

# start in state halted and grab the current time
state = "H"
last_key_press_time = rospy.Time.now()


global turn; global timer; global ang_direct;
turn = False
timer = rospy.Time.now()
ang_direct = 1
# set rate
rate = rospy.Rate(10)

#

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():

   # print out the current state and time since last key press
    print_state()

   # publish cmd_vel from here 
    t = Twist()


    velocity_vector = dic_velocity_vector[state]

    if state == "S":
        t.linear.x = 0.1 + radius
        t.angular.z = max((math.pi/4 - radius) * 0.9, 0)
        radius+=0.001
    elif state == "Z":
        curr_time = rospy.Time.now()
        if curr_time.to_sec() - timer.to_sec() > 4 :
            turn = not turn
            timer = curr_time
            if turn:
                ang_direct = -1 * ang_direct
        if turn:
            t.linear.x = 0
            t.angular.z = ang_direct * ANGULAR_SPEED * velocity_vector[1]
        else:
            t.linear.x = LINEAR_SPEED * velocity_vector[0]
            t.angular.z = 0
        print(str(turn) + " " + str(t.angular.z))
    else:
        t.linear.x, t.angular.z = LINEAR_SPEED * velocity_vector[0], ANGULAR_SPEED * velocity_vector[1]

   # one idea: use vector-2 to represent linear and angular velocity
   # velocity_vector = [linear_component, angular_component]
   # then represent:
   # twist.linear.x = LINEAR_SPEED * linear_component
   # twist.angular.z = ANGULAR_SPEED * angular_component 
   # where for example:
   # LINEAR_SPEED = 0.2, ANGULAR_SPEED = pi/4
   # velocity_vector = [1, 0] for positive linear and no angular movement
   # velocity_vector = [-1, 1] for negative linear and positive angular movement
   # we can then create a dictionary state: movement_vector to hash the current position to get the movement_vector
   # in order to get the zig zag and spiral motion you could you something like this:
   # twist.linear.x = LINEAR_SPEED * linear_component * linear_transform
   # twist.angular.z = ANGULAR_SPEED * angular_component * angular_transform
   # where the [linear_transform, angular_transform] is derived from another source that is based on the clock
   # now you can change the velocity of the robot at every step of the program based on the state and the time
    cmd_vel_pub.publish(t)

   # run at 10hz
    rate.sleep()
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':

  rospy.init_node('keys')
  key_pub = rospy.Publisher('keys', String, queue_size = 1)
  valid_commands = ["L", "R", "F", "B", "H", "S", "Z"]
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():

      command = input("instruction from user input: ").upper()
      if command in valid_commands:
        msg = String(command)
        key_pub.publish(msg)
      rate.sleep()
      
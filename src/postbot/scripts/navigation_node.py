#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, BoxGoal
from threading import Condition


rospy.init_node("navigation", anonymous=False)
box_color = ' '
rate = rospy.Rate(1)

# The node has to get the box goal message first and then it searches for the box status
sequencing = Condition()
flag = False
second_flag = False
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']

def callback(data):
    global flag, box_color
   # with sequencing:
    box_color = data.color
    flag = True
    rospy.loginfo(f"Box full is the color: {box_color}")

#assigned_color = colors.index(box_color) #index of the position

box_status = BoxInfo()

rospy.Subscriber('/box_goal', BoxGoal, callback)

def box_status_callback(data):
    global box_status, second_flag
    #with sequencing:
        #while not flag:
          #sequencing.wait()
    box_status = data
    second_flag = True
    rospy.loginfo(f"Current box status: {box_status.status}")


rospy.Subscriber('/box_status', BoxInfo, box_status_callback)


pub = rospy.Publisher("/box_status", BoxInfo, queue_size=10)

while not rospy.is_shutdown():
    #with sequencing:
    rospy.loginfo(f"Values of flag : {flag} and second_flag: {second_flag}")
    if flag and second_flag:
            rospy.loginfo("Updating boxes status:")
            current_status = list(box_status.status)
            i = colors.index(box_color)
            rospy.loginfo(f"Position of color: {i}")
            current_status[i] = 1
            box_status.status = current_status
            pub.publish(box_status)
            flag = False
            second_flag = False
            rospy.loginfo("Updated boxes status:")

    rospy.sleep(5)




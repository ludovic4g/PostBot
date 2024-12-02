#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, BoxGoal
from postbot.srv import reset_boxes

rospy.init_node("navigation", anonymous=False)
box_color = ' '
rate = rospy.Rate(1)

# The node has to get the box goal message first and then it searches for the box status
flag = False
second_flag = False
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']

def callback(data):
    global flag, box_color
    box_color = data.color
    flag = True
    rospy.loginfo(f"Box full is the color: {box_color}")

#assigned_color = colors.index(box_color) #index of the position

box_status = BoxInfo()

rospy.Subscriber('/box_goal', BoxGoal, callback)

def box_status_callback(data):
    global box_status, second_flag
    box_status = data
    second_flag = True


rospy.Subscriber('/box_status', BoxInfo, box_status_callback)


pub = rospy.Publisher("/box_status", BoxInfo, queue_size=10)

rospy.wait_for_service('/reset_boxes')
reset_boxes_service = rospy.ServiceProxy('/reset_boxes', reset_boxes)

while not rospy.is_shutdown():
    if flag and second_flag:
            rospy.loginfo("Updating boxes status:")
            current_status = list(box_status.status)
            i = colors.index(box_color)
            current_status[i] = 1
            rospy.loginfo(f"Current box status: {current_status}")
            empty_box = [index for index, value in enumerate(current_status) if value == 0]
            rospy.loginfo(f"Number of available boxes: {len(empty_box)}")
            if len(empty_box) == 0:
                try:
                     response = reset_boxes_service()
                     if response.done:
                          rospy.loginfo("Reset service sent successfully")
                          #break
                except rospy.ServiceException as e:
                    rospy.logerr("Service call failed")
            else:
                box_status.status = current_status
                pub.publish(box_status)
                flag = False
                second_flag = False
                rospy.loginfo("Updated boxes status:")

    rospy.sleep(5)




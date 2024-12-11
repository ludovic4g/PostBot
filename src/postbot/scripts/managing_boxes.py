#!/usr/bin/python3
import rospy
from postbot.msg import BoxGoal, BoxInfo, MarbleInfo
from threading import Condition

flag = False
box_flag = False
sequencing = Condition()
rospy.init_node('managing_boxes', anonymous=False)

colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
box_goal = None

def marble_callback(data):
    global flag, new_box_color
    with sequencing:
        new_box_color = data.color
        flag = True
        sequencing.notify_all()

def boxstatus_callback(data):
    global box_goal, box_flag, new_box_color
    with sequencing:
        while not flag:
            sequencing.wait()
        color_position = colors.index(new_box_color)
        box_goal = BoxGoal()
        box_goal.x = data.x[color_position]
        box_goal.y = data.y[color_position]
        box_goal.color = colors[color_position]
        box_flag = True
        sequencing.notify_all()

rospy.Subscriber('/current_marble', MarbleInfo, marble_callback)
rospy.Subscriber('/box_status', BoxInfo, boxstatus_callback)

pub = rospy.Publisher('/box_goal', BoxGoal, queue_size=10)

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    with sequencing:
        if flag and box_flag:
            pub.publish(box_goal)
            flag = False
            box_flag = False
    rospy.sleep(5)

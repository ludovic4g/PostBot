#!/usr/bin/python3
import rospy
from postbot.msg import BoxGoal, BoxInfo, MarbleInfo
from threading import Condition
from visualization_msgs.msg import MarkerArray

flag = False
box_flag = False
sequencing = Condition()  # Thread per il sequencing dei callback
rospy.init_node('managing_boxes', anonymous=False)

rate = rospy.Rate(1)
rospy.loginfo("Initializing node")
new_box_color = ' '
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
box_goal = None
box_status = None

def box_goal_callback(data):
    global flag, new_box_color, box_goal
    with sequencing:
        new_box_color = data.color
        box_goal = data
        flag = True
        sequencing.notify_all()

def box_status_callback(data):
    global box_status, box_flag
    with sequencing:
        box_status = data
        box_flag = True
        sequencing.notify_all()

rospy.Subscriber('/box_goal', BoxGoal, box_goal_callback)
rospy.Subscriber('/box_status', BoxInfo, box_status_callback)

# Pubblica le scatole come MarkerArray
boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)

# Ciclo principale per aggiornare lo stato delle scatole
while not rospy.is_shutdown():
    with sequencing:
        if flag and box_flag:
            rospy.loginfo(f"Managing box goal for color: {new_box_color}")
            # Logica di aggiornamento scatole (da implementare se necessario)
            flag = False
            box_flag = False
    rate.sleep()

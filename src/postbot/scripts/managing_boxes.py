#!/usr/bin/python3
import rospy
from postbot.msg import BoxGoal, BoxInfo, MarbleInfo
from threading import Condition

flag = False
box_flag = False
sequencing = Condition() # Threading for the right sequencing of the callbacks
rospy.init_node('managing_boxes', anonymous=False)

rate = rospy.Rate(1)
rospy.loginfo("Initializing node")
new_box_color = ' '
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
box_goal = None

#rospy.loginfo("Prima callback 1")

def callback(data):
    global new_box_color, flag
    with sequencing:
        rospy.loginfo(f"Color of the marble : {data.color}")
        new_box_color = data.color
        flag = True
        rospy.loginfo(f"Box assigned color 1: {new_box_color}")
        #box_flag = False
        sequencing.notify_all()



def boxstatus_callback(data):
    rospy.loginfo("Sono nel secondo callback")
    global box_goal, box_flag
    with sequencing:
        while not flag:
            sequencing.wait()
        color_position = colors.index(new_box_color) 
        rospy.loginfo(f"color position? : {color_position}")
        box_goal = BoxGoal()
        box_goal.x = data.x[color_position]
        box_goal.y = data.y[color_position]
        box_goal.color = colors[color_position]
        box_flag= True
        rospy.loginfo(f"Box: {box_goal}")
            #flag = False
        sequencing.notify_all()


# Subscribes to the topic /current_marble to take the color and its position
rospy.Subscriber('/current_marble', MarbleInfo, callback)


# Subscribes to the topic /box_status to find box needing delivery
rospy.Subscriber('/box_status', BoxInfo, boxstatus_callback)

rospy.loginfo(f"Box assigned color 2: {new_box_color}")

# Publish the coordinates of the box that has to be delivered
pub = rospy.Publisher('/box_goal', BoxGoal, queue_size=10)


while not rospy.is_shutdown():
    #rospy.loginfo(f"Waiting for box_flag: {box_flag} and flag: {flag}")
    with sequencing:
        if flag and box_flag:
            rospy.loginfo(f"Box to deliver: {box_goal}")
            rospy.loginfo(f"Publishing into /box_goal topic")
            pub.publish(box_goal)
            rospy.loginfo(f"Published into /box_goal topic")
            flag = False
            box_flag = False

    rospy.sleep(5)






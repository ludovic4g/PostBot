#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, BoxGoal
from postbot.srv import reset_boxes
from visualization_msgs.msg import Marker, MarkerArray
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import Twist

rospy.init_node("navigation", anonymous=False)
box_color = ' '
rate = rospy.Rate(1)
boxpub = None
robotpub = None

# IL ROBOT DEVE CONSEGNARE LA PALLINA ALLA BOX CON LO STESSO COLORE
# FINO A QUANDO NON HA CONSEGNATO NON PUO' RITORNARE ALLO SPAWN MARBLE PER LA NUOVA PALLINA

# The node has to get the box goal message first and then it searches for the box status
flag = False
second_flag = False
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']

def update_box_marker(i):
    global boxpub
    box = Marker()
    box.header.frame_id = "world" #?
    box.id = i
    box.ns ="boxes"
    box.type = Marker.CUBE
    box.action = Marker.ADD
    box.pose.position.x = x
    box.pose.position.y = y
    box.pose.position.z = 0.25
    box.scale.x = 0.5
    box.scale.y = 0.5
    box.scale.z = 0.5

    #colore grigio perchè usato
    box.color.r = 0.5
    box.color.g = 0.5
    box.color.b = 0.5

    box.color.a = 1.0

    boxpub.publish(box)

    rospy.loginfo("Box full updated")
     
def navigate_to_goal(x, y):
    rospy.loginfo("Navigating to box goal:")
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    velocity_msg = Twist()
    rate = rospy.Rate(10)

    velocity_msg.linear.x = 1.0
    velocity_msg.angular.z = 0.0
    for _ in range(10):
        pub.publish(velocity_msg)
        robot_marker(x, y)
        rate.sleep()

    velocity_msg.linear.x = 0.0
    pub.publish(velocity_msg)

    rospy.wait_for_service('/turtle1/teleport_absolute')
    teleport_service = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

    teleport_service(x,y,0.0)




def robot_marker(x, y):
    global robotpub
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id="0"
    marker.type= Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    robotpub.publish(marker)
    rospy.loginfo("Published robot marker")
    
    

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

robotpub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
pub = rospy.Publisher("/box_marker", MarkerArray, queue_size=10)

boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)

rospy.wait_for_service('/reset_boxes')
reset_boxes_service = rospy.ServiceProxy('/reset_boxes', reset_boxes)

while not rospy.is_shutdown():
    if flag and second_flag:
            rospy.loginfo("Updating boxes status:")
            current_status = list(box_status.status)
            i = colors.index(box_color)
            goal_x = box_status[i].x
            goal_y = box_status[i].y

            navigate_to_goal(goal_x,goal_y)
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
                update_box_marker(i)
                flag = False
                second_flag = False
                rospy.loginfo("Updated boxes status:")

    rospy.sleep(5)




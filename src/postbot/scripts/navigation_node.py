#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, BoxGoal
from postbot.srv import reset_boxes
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

rospy.init_node("navigation", anonymous=False)
box_color = ' '
rate = rospy.Rate(1)
boxpub = None
robotpub = None

colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']

flag = False
second_flag = False
box_status = BoxInfo()

def update_box_marker(i, box_status):
    global boxpub
    marker_array = MarkerArray()
    for j in range(len(box_status.colors)):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = j
        marker.ns = "boxes"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = box_status.x[j]
        marker.pose.position.y = box_status.y[j]
        marker.pose.position.z = 0.25
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        if j == i:
            # Box piena = grigio
            marker.color.r = 0.5; marker.color.g = 0.5; marker.color.b = 0.5
        else:
            color = box_status.colors[j]
            if color == 'red':
                marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
            elif color == 'blue':
                marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0
            elif color == 'green':
                marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
            elif color == 'yellow':
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0
            elif color == 'white':
                marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 1.0
            elif color == 'purple':
                marker.color.r = 0.5; marker.color.g = 0.0; marker.color.b = 0.5

        marker.color.a = 1.0
        marker_array.markers.append(marker)

    boxpub.publish(marker_array)

def navigate_to_goal(x, y):
    # Pubblica un goal per move_base
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0
    goal_pub.publish(goal)

def robot_marker(x, y):
    global robotpub
    marker = Marker()
    marker.header.frame_id = "map"
    marker.id = 0
    marker.ns = "robot"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    robotpub.publish(marker)

def box_goal_callback(data):
    global flag, box_color
    box_color = data.color
    flag = True

def box_status_callback(data):
    global box_status, second_flag
    box_status = data
    second_flag = True

rospy.Subscriber('/box_goal', BoxGoal, box_goal_callback)
rospy.Subscriber('/box_status', BoxInfo, box_status_callback)

robotpub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
pub = rospy.Publisher("/box_status", BoxInfo, queue_size=10)
boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)

rospy.wait_for_service('/reset_boxes')
reset_boxes_service = rospy.ServiceProxy('/reset_boxes', reset_boxes)

while not rospy.is_shutdown():
    if flag and second_flag:
        current_status = list(box_status.status)
        i = colors.index(box_color)
        goal_x = box_status.x[i]
        goal_y = box_status.y[i]

        # Aggiorna posizione robot (marker)
        robot_marker(goal_x, goal_y)

        # Naviga verso la scatola con move_base
        navigate_to_goal(goal_x, goal_y)

        # Aggiorna stato scatola
        current_status[i] = 1
        empty_box = [index for index, value in enumerate(current_status) if value == 0]

        if len(empty_box) == 0:
            response = reset_boxes_service()
            if response.done:
                rospy.loginfo("Reset service sent successfully")
        else:
            box_status.status = current_status
            pub.publish(box_status)
            update_box_marker(i, box_status)
            flag = False
            second_flag = False
    rospy.sleep(5)

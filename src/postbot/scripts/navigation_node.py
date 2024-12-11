#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, BoxGoal
from postbot.srv import reset_boxes
from visualization_msgs.msg import Marker, MarkerArray
from turtlesim.srv import TeleportAbsolute
from geometry_msgs.msg import PoseStamped

# Inizializza il nodo
rospy.init_node("navigation", anonymous=False)
box_color = ' '
rate = rospy.Rate(1)
boxpub = None
robotpub = None
marblepub = None

# Colori delle scatole
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']

# Flags per sincronizzare i messaggi
flag = False
second_flag = False

def update_box_marker(i, x, y, box_status):
    global boxpub

    # Crea un nuovo MarkerArray per rappresentare tutte le scatole
    marker_array = MarkerArray()

    # Itera su tutte le scatole definite in box_status
    for j in range(len(box_status.colors)):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.id = j
        marker.ns = "boxes"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Posizione della scatola
        marker.pose.position.x = box_status.x[j]
        marker.pose.position.y = box_status.y[j]
        marker.pose.position.z = 0.25
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Colore basato sullo stato della scatola
        if j == i:  # Se è la scatola da aggiornare
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5  # Grigio per scatola piena
        else:  # Altrimenti usa il colore originale
            color = box_status.colors[j]
            if color == 'red':
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            elif color == 'blue':
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
            elif color == 'green':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif color == 'yellow':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif color == 'white':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            elif color == 'purple':
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 0.5

        # Tutte le scatole devono essere visibili
        marker.color.a = 1.0

        # Aggiungi il marker al MarkerArray
        marker_array.markers.append(marker)

    # Pubblica il MarkerArray aggiornato
    boxpub.publish(marker_array)
    rospy.loginfo(f"Updated box {i} to gray (full) in MarkerArray")

def navigate_to_goal(x, y):
    rospy.loginfo("Navigating to box goal:")
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.w = 1.0
    pub.publish(goal)

def robot_marker(x, y):
    global robotpub
    marker = Marker()
    marker.header.frame_id = "world"
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
    rospy.loginfo("Published robot marker")

def marble_marker(x, y, color):
    global marblepub
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = 0
    marker.ns = "marble"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    # Imposta il colore della biglia
    if color == 'red':
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif color == 'blue':
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif color == 'green':
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'yellow':
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif color == 'white':
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
    elif color == 'purple':
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5

    marker.color.a = 1.0
    marblepub.publish(marker)
    rospy.loginfo("Published marble marker")

def callback(data):
    global flag, box_color
    box_color = data.color
    flag = True
    rospy.loginfo(f"Box full is the color: {box_color}")

box_status = BoxInfo()
rospy.Subscriber('/box_goal', BoxGoal, callback)

def box_status_callback(data):
    global box_status, second_flag
    box_status = data
    second_flag = True

rospy.Subscriber('/box_status', BoxInfo, box_status_callback)

robotpub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
marblepub = rospy.Publisher("/marble_marker", Marker, queue_size=10)
pub = rospy.Publisher("/box_status", BoxInfo, queue_size=10)
boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)

rospy.wait_for_service('/reset_boxes')
reset_boxes_service = rospy.ServiceProxy('/reset_boxes', reset_boxes)

while not rospy.is_shutdown():
    if flag and second_flag:
        rospy.loginfo("Updating boxes status:")
        current_status = list(box_status.status)
        i = colors.index(box_color)
        goal_x = box_status.x[i]
        goal_y = box_status.y[i]

        # Simula la raccolta della biglia
        marble_x = 1.0  # Posizione della biglia (esempio)
        marble_y = 2.0
        marble_color = box_color  # Colore associato

        # Pubblica la biglia
        marble_marker(marble_x, marble_y, marble_color)

        # Simula il movimento del robot
        robot_marker(marble_x, marble_y)
        rospy.loginfo("Robot reached the marble")
        rospy.sleep(1)  # Pausa per simulare raccolta

        # Simula il movimento verso la scatola
        navigate_to_goal(goal_x, goal_y)
        robot_marker(goal_x, goal_y)

        # Aggiorna la scatola come piena
        update_box_marker(i, goal_x, goal_y, box_status)
        rospy.loginfo("Marble delivered to box")

        flag = False
        second_flag = False

    rospy.sleep(5)

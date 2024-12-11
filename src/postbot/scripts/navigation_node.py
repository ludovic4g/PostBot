#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, MarbleInfo
from postbot.srv import reset_boxes
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Inizializzazione del nodo
rospy.init_node("navigation", anonymous=False)

# Publisher per i marker e per i comandi di velocità del turtle
robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
marble_marker_pub = rospy.Publisher("/marble_marker", Marker, queue_size=10)
cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
box_status_pub = rospy.Publisher("/box_status", BoxInfo, queue_size=10)

# Service Proxy per resettare le scatole
rospy.wait_for_service('/reset_boxes')
reset_boxes_service = rospy.ServiceProxy('/reset_boxes', reset_boxes)

# Variabili globali
current_pose = Pose()
current_state = "IDLE"  # Stati: IDLE, MOVING_TO_MARBLE, MOVING_TO_BOX
current_marble = None
box_status = BoxInfo()
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
tolerance = 0.5  # Tolleranza per raggiungere il goal (adattata per Turtlesim)

def pose_callback(data):
    global current_pose
    current_pose = data
    update_robot_marker(current_pose)

def current_marble_callback(data):
    global current_marble, current_state
    if current_state == "IDLE":
        current_marble = data
        rospy.loginfo(f"Nuova marble: {current_marble.color} a ({current_marble.x}, {current_marble.y})")
        current_state = "MOVING_TO_MARBLE"

def box_status_callback(data):
    global box_status
    box_status = data

def update_robot_marker(pose):
    marker = Marker()
    marker.header.frame_id = "world"  # 'world' frame per Turtlesim
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.position.z = 0.1
    marker.pose.orientation.w = 1.0  # Orientamento neutro
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    robot_marker_pub.publish(marker)

def move_turtle(target_x, target_y):
    twist = Twist()
    # Calcolo della distanza e dell'angolo verso il target
    distance = math.sqrt((target_x - current_pose.x)**2 + (target_y - current_pose.y)**2)
    angle_to_target = math.atan2(target_y - current_pose.y, target_x - current_pose.x)
    angle_diff = angle_to_target - current_pose.theta

    # Normalizzazione dell'angolo_diff tra -pi e pi
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    # Controllo proporzionale per l'orientamento
    if abs(angle_diff) > 0.05:
        twist.angular.z = 2.0 * angle_diff
        twist.linear.x = 0.0
    else:
        twist.angular.z = 0.0
        # Controllo proporzionale per la distanza
        if distance > tolerance:
                twist.linear.x = 1.5 * distance
        else:
                twist.linear.x = 0.0

    # Limitare i valori di velocità
    twist.linear.x = max(min(twist.linear.x, 2.0), -2.0)
    twist.angular.z = max(min(twist.angular.z, 2.0), -2.0)

    cmd_vel_pub.publish(twist)

def check_reached(target_x, target_y):
    distance = math.sqrt((target_x - current_pose.x)**2 + (target_y - current_pose.y)**2)
    return distance < tolerance

def manage_movement():
    global current_state, current_marble, box_status

    if current_state == "MOVING_TO_MARBLE":
        if current_marble:
            move_turtle(current_marble.x, current_marble.y)
            if check_reached(current_marble.x, current_marble.y):
                rospy.loginfo(f"Pallina {current_marble.color} raccolta a ({current_marble.x}, {current_marble.y})")
                # Rimuovere la marble dal suo posto
                remove_marble_marker(current_marble)
                # Passare allo stato di movimento verso la scatola
                current_state = "MOVING_TO_BOX"
    elif current_state == "MOVING_TO_BOX":
        if current_marble:
            box_index = colors.index(current_marble.color)
            target_box_x = box_status.x[box_index]
            target_box_y = box_status.y[box_index]
            move_turtle(target_box_x, target_box_y)
            if check_reached(target_box_x, target_box_y):
                rospy.loginfo(f"Consegna pallina {current_marble.color} nella scatola a ({target_box_x}, {target_box_y})")
                # Aggiungere la marble alla scatola
                add_marble_to_box(current_marble, box_index)
                # Incrementare lo stato della scatola
                current_boxes = list(box_status.status)
                current_boxes[box_index] = 1
                box_status.status = current_boxes
                box_status_pub.publish(box_status)
                remove_marble_from_box(current_marble.color)
                # Resettare la marble corrente
                current_marble = None
                # Verificare se tutte le scatole sono piene
                if all(status == 1 for status in box_status.status):
                    rospy.loginfo("Tutte le scatole sono piene. Reset in corso...")
                    try:
                        response = reset_boxes_service()
                        if response.done:
                            rospy.loginfo("Reset delle scatole completato.")
                            current_state = "IDLE"
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Service call failed: {e}")
                else:
                    current_state = "IDLE"
    elif current_state == "IDLE":
        pass  # Attendere nuovi obiettivi

def remove_marble_marker(marble):
    # Rimuovere la marble dalla sua posizione originale
    marker_remove = Marker()
    marker_remove.header.frame_id = "world"
    # marker_remove.header.stamp = rospy.Time.now()
    marker_remove.ns = "marbles"
    if marble.color == 'red':
        marker_remove.id = 11 
    if marble.color == 'blue':
        marker_remove.id = 12 
    if marble.color == 'green':
        marker_remove.id = 13 
    if marble.color == 'white':
        marker_remove.id = 14 
    if marble.color == 'purple':
        marker_remove.id = 15 
    if marble.color == 'yellow':
        marker_remove.id = 16
    rospy.loginfo(f"Id della biglia da levare: {marker_remove.id}")
    marker_remove.action = Marker.DELETE
    marble_marker_pub.publish(marker_remove)
    rospy.loginfo(f"Marble {marble.color} rimossa dalla posizione ({marble.x}, {marble.y})")

def add_marble_to_box(marble, box_index):
    marker_add = Marker()
    marker_add.header.frame_id = "world"
    marker_add.header.stamp = rospy.Time.now()
    marker_add.ns = "marbles"
    marker_add.id = box_index  # Assicurarsi che l'ID sia unico per ogni scatola
    marker_add.type = Marker.SPHERE
    marker_add.action = Marker.ADD
    marker_add.pose.position.x = box_status.x[box_index]
    marker_add.pose.position.y = box_status.y[box_index]
    marker_add.pose.position.z = 0.1
    marker_add.pose.orientation.w = 1.0
    marker_add.scale.x = 0.2
    marker_add.scale.y = 0.2
    marker_add.scale.z = 0.2

    # Impostare il colore basato sul colore della marble
    color = marble.color
    if color == 'red':
        marker_add.color.r = 1.0
        marker_add.color.g = 0.0
        marker_add.color.b = 0.0
    elif color == 'blue':
        marker_add.color.r = 0.0
        marker_add.color.g = 0.0
        marker_add.color.b = 1.0
    elif color == 'green':
        marker_add.color.r = 0.0
        marker_add.color.g = 1.0
        marker_add.color.b = 0.0
    elif color == 'white':
        marker_add.color.r = 1.0
        marker_add.color.g = 1.0
        marker_add.color.b = 1.0
    elif color == 'purple':
        marker_add.color.r = 0.5
        marker_add.color.g = 0.0
        marker_add.color.b = 0.5
    elif color == 'yellow':
        marker_add.color.r = 1.0
        marker_add.color.g = 1.0
        marker_add.color.b = 0.0

    marker_add.color.a = 1.0
    marble_marker_pub.publish(marker_add)
    rospy.loginfo(f"Marble {color} spostata nella scatola a ({marker_add.pose.position.x}, {marker_add.pose.position.y})")

def remove_marble_from_box(color):
    marker_remove = Marker()
    marker_remove.header.frame_id = "world"
    marker_remove.header.stamp = rospy.Time.now()
    marker_remove.ns = "marbles"
    if color == 'red':
        marker_remove.id = 11 
    if color == 'blue':
        marker_remove.id = 12 
    if color == 'green':
        marker_remove.id = 13 
    if color == 'white':
        marker_remove.id = 14 
    if color == 'purple':
        marker_remove.id = 14 
    if color == 'yellow':
        marker_remove.id = 16
    rospy.loginfo(f"Marble id that have to be remove: {marker_remove.id}")
    marker_remove.action = Marker.DELETE
    marble_marker_pub.publish(marker_remove)
    rospy.loginfo(f"Marble consegnata alla scatola {color}")

def main():
    # Sottoscrizioni
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/current_marble', MarbleInfo, current_marble_callback)
    rospy.Subscriber('/box_status', BoxInfo, box_status_callback)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        manage_movement()
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

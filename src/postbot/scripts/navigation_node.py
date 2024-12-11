#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, BoxGoal, MarbleInfo
from postbot.srv import reset_boxes
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Inizializzazione del nodo
rospy.init_node("navigation", anonymous=False)

# Publisher per i marker e per i comandi di velocità del turtle
robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
box_marker_pub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)
cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
box_status_pub = rospy.Publisher("/box_status", BoxInfo, queue_size=10)

# Service Proxy per resettare le scatole
rospy.wait_for_service('/reset_boxes')
reset_boxes_service = rospy.ServiceProxy('/reset_boxes', reset_boxes)

# Variabili globali
current_pose = Pose()
goal_pose = None
current_state = "IDLE"  # Stati: IDLE, MOVING_TO_MARBLE, MOVING_TO_BOX
box_goal = None
box_status = BoxInfo()
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
tolerance = 0.1  # Tolleranza per raggiungere il goal

def pose_callback(data):
    global current_pose
    current_pose = data
    update_robot_marker(current_pose)

def box_goal_callback(data):
    global goal_pose, current_state, box_goal
    box_goal = data
    # Se siamo IDLE, iniziamo a muoverci verso la pallina
    if current_state == "IDLE":
        goal_pose = MarbleInfo()
        goal_pose.x = box_goal.x
        goal_pose.y = box_goal.y
        goal_pose.color = box_goal.color
        current_state = "MOVING_TO_MARBLE"
        rospy.loginfo(f"Nuovo obiettivo: Pallina {box_goal.color} a ({goal_pose.x}, {goal_pose.y})")

def box_status_callback(data):
    global box_status
    box_status = data

def update_robot_marker(pose):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.position.z = 0.1
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

    # Se non siamo diretti verso il target, ruotiamo
    if abs(angle_diff) > 0.05:
        twist.angular.z = 2.0 * angle_diff
    else:
        twist.angular.z = 0.0
        # Se siamo orientati correttamente, muoviamoci avanti
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
    global current_state, goal_pose, box_goal, box_status

    if current_state == "MOVING_TO_MARBLE":
        if goal_pose:
            move_turtle(goal_pose.x, goal_pose.y)
            if check_reached(goal_pose.x, goal_pose.y):
                rospy.loginfo(f"Pallina {goal_pose.color} raccolta a ({goal_pose.x}, {goal_pose.y})")
                # Aggiornare lo stato della scatola corrispondente
                box_index = colors.index(goal_pose.color)
                box_status.status[box_index] += 1
                box_status_pub.publish(box_status)
                # Passare allo stato di movimento verso la scatola
                box_goal = None  # Reset
                current_state = "MOVING_TO_BOX"
    elif current_state == "MOVING_TO_BOX":
        if box_goal:
            target_box_x = box_goal.x
            target_box_y = box_goal.y
            move_turtle(target_box_x, target_box_y)
            if check_reached(target_box_x, target_box_y):
                rospy.loginfo(f"Consegna pallina {box_goal.color} nella scatola a ({target_box_x}, {target_box_y})")
                # Verificare se tutte le scatole sono piene
                if all(status >= 1 for status in box_status.status):
                    rospy.loginfo("Tutte le scatole sono piene. Reset in corso...")
                    response = reset_boxes_service()
                    if response.done:
                        rospy.loginfo("Reset delle scatole completato.")
                        current_state = "IDLE"
                else:
                    current_state = "IDLE"
    elif current_state == "IDLE":
        pass  # Attendere nuovi obiettivi

def main():
    # Sottoscrizioni
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.Subscriber('/box_goal', BoxGoal, box_goal_callback)
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

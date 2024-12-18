#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo, MarbleInfo
from postbot.srv import reset_boxes
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, TeleportAbsoluteRequest
import math

robot_pose = False

rospy.init_node("navigation", anonymous=False)

robot_marker_pub = rospy.Publisher("/robot_marker", Marker, queue_size=10)
marble_marker_pub = rospy.Publisher("/marble_marker", Marker, queue_size=10)
cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
box_status_pub = rospy.Publisher("/box_status", BoxInfo, queue_size=10)
boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)

# service call
rospy.wait_for_service('/reset_boxes')
reset_boxes_service = rospy.ServiceProxy('/reset_boxes', reset_boxes)


current_pose = Pose()
# Marble states for navigation: IDLE, MOVING_TO_MARBLE, MOVING_TO_BOX
current_state = "IDLE"  
current_marble = None
box_status = BoxInfo()
colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
tolerance = 0.5 

# callback for turtlesim position
def pose_callback(data):
    global current_pose
    current_pose = data
    update_robot_marker(current_pose)


# callback for checking current marble to navigate towards it
def current_marble_callback(data):
    global current_marble, current_state
    if current_state == "IDLE":
        current_marble = data
        rospy.loginfo(f"Nuova marble: {current_marble.color} a ({current_marble.x}, {current_marble.y})")
        current_state = "MOVING_TO_MARBLE"

# callback for checking box status, needed for changing color 
def box_status_callback(data):
    global box_status
    box_status = data
    update_boxes(box_status)


# function to update turtlesim configuration
def update_robot_marker(pose):
    global robot_pose
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = 0
    marker.ns = "postbot"
    marker.type = Marker.SPHERE 
    marker.action = Marker.ADD

    # if not set, take position from yaml file
    #rospy.loginfo(f"robot_pose value: {robot_pose}")
    if not robot_pose:
        initial_pose = rospy.get_param('robot_initial_pose', 'src/postbot/config/robot_par.yaml')
        pose.x = initial_pose['x']
        pose.y = initial_pose['y']
        robot_pose = True
    
    marker.pose.position.x = pose.x
    marker.pose.position.y = pose.y
    marker.pose.position.z = 0.1
    marker.pose.orientation.w = 1.0 
    marker.scale.x = 0.5
    marker.scale.y = 0.2
    marker.scale.z = 0.5
    marker.color.r = 0.5
    marker.color.g = 1
    marker.color.b = 1

    marker.color.a = 1.0
    robot_marker_pub.publish(marker)

def move_turtle(target_x, target_y):
    twist = Twist()
    
    # distance calculation between turtle sim and goal
    distance = math.sqrt((target_x - current_pose.x)**2 + (target_y - current_pose.y)**2)
    angle_to_target = math.atan2(target_y - current_pose.y, target_x - current_pose.x)
    angle_diff = angle_to_target - current_pose.theta
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    # calculation of distance and toleration
    if abs(angle_diff) > 0.05:
        twist.angular.z = 2.0 * angle_diff
        twist.linear.x = 0.0
    else:
        twist.angular.z = 0.0
        if distance > tolerance:
                twist.linear.x = 1.5 * distance
        else:
                twist.linear.x = 0.0

    twist.linear.x = max(min(twist.linear.x, 2.0), -2.0)
    twist.angular.z = max(min(twist.angular.z, 2.0), -2.0)

    cmd_vel_pub.publish(twist)

def check_reached(target_x, target_y):
    distance = math.sqrt((target_x - current_pose.x)**2 + (target_y - current_pose.y)**2)
    return distance < tolerance

def manage_movement():
    global current_state, current_marble, box_status, robot_pose

    if current_state == "MOVING_TO_MARBLE":
        # moves to marble and then 'takes it'
        if current_marble:
            move_turtle(current_marble.x, current_marble.y)
            if check_reached(current_marble.x, current_marble.y):
                rospy.loginfo(f"{current_marble.color} marble taken in position ({current_marble.x}, {current_marble.y})")
                remove_marble_marker(current_marble)
                current_state = "MOVING_TO_BOX"
    elif current_state == "MOVING_TO_BOX":
        if current_marble:
            box_index = colors.index(current_marble.color)
            target_box_x = box_status.x[box_index]
            target_box_y = box_status.y[box_index]
            move_turtle(target_box_x, target_box_y)
            if check_reached(target_box_x, target_box_y):
                rospy.loginfo(f"Delivering {current_marble.color} marble to box in position ({target_box_x}, {target_box_y})")
                # Update boxes status
                current_boxes = list(box_status.status)
                current_boxes[box_index] = 1
                rospy.loginfo(f"Current status of the boxes: {current_boxes}")
                box_status.status = current_boxes
                box_status_pub.publish(box_status)
                #Change color to grey when they are full
                update_boxes(box_status)

                # waiting for new spawned marble
                current_marble = None
                # If all boxes are full
                if all(status == 1 for status in box_status.status):
                    rospy.loginfo("Boxes full. Calling reset service.")
                    try:
                        response = reset_boxes_service()
                        rospy.loginfo("")
                        robot_pose = False
                        if response.done:
                            initial_pose = rospy.get_param('robot_initial_pose', 'src/postbot/config/robot_par.yaml')
                            pose = Pose()
                            pose.x = initial_pose['x']
                            pose.y = initial_pose['y']

                            teleport = TeleportAbsoluteRequest()
                            teleport.x = pose.x
                            teleport.y = pose.y
                            teleport.theta = 0.0

                            rospy.wait_for_service('/turtle1/teleport_absolute')
                            teleport_proxy = rospy.ServiceProxy('turtle1/teleport_absolute', TeleportAbsolute)
                            teleport_response = teleport_proxy(teleport.x, teleport.y, teleport.theta)
                            rospy.loginfo("Reset complete.")
                            current_state = "IDLE"
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Service call failed: {e}")
                else:
                    current_state = "IDLE"
    elif current_state == "IDLE":
        pass 

def remove_marble_marker(marble):
    # When the marble is taken, it is removed from the map
    marker_remove = Marker()
    marker_remove.header.frame_id = "world"
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
    rospy.loginfo(f"MId of the marble to remove: {marker_remove.id}")
    marker_remove.action = Marker.DELETE
    marble_marker_pub.publish(marker_remove)
    rospy.loginfo(f"Marble {marble.color} removed from ({marble.x}, {marble.y})")


# for every full box, its color changes to grey
def update_boxes(box):
    global boxpub
    boxes = MarkerArray()

    for i, (color, status) in enumerate(zip(box.colors, box.status)):
        marker = Marker()
        marker.header.frame_id = "world" 
        marker.ns = "boxes"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = box.x[i]
        marker.pose.position.y = box.y[i]
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0 
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        if status == 1:
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5
        else:
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
            elif color == 'white':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
            elif color == 'purple':
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 0.5
            elif color == 'yellow':
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0

        marker.color.a = 1.0
        boxes.markers.append(marker)
    
    boxpub.publish(boxes)


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

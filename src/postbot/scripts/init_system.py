#!/usr/bin/python3
import rospy
import random
from geometry_msgs.msg import Pose
from postbot.msg import BoxInfo
from postbot.srv import reset_boxes, reset_boxesResponse
from visualization_msgs.msg import Marker, MarkerArray

boxpub = None
robotpub = None

def spawn_robot():
    global robotpub
    robot_name = "postbot"
    # Correzione del parametro: robot_initial_pose (prima era robor_initial_pose)
    robot_initial_pose = rospy.get_param('robot_initial_pose', 'src/postbot/config/robot_par.yaml')

    model_path = rospy.get_param('model_path', '/home/ludoludo/postbot/src/postbot/urdf/postbot.urdf')

    with open(model_path, 'r') as urdf_file:
        robot_urdf = urdf_file.read()
        rospy.loginfo("Urdf Model read")

    rospy.loginfo("Spawned in Gazebo")

    marker = Marker()
    # Cambiato frame_id da "world" a "map"
    marker.header.frame_id = "map"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = 0.5
    marker.pose.position.y = 0.5
    marker.pose.position.z = 0.25
    marker.scale.x = 0.5
    marker.scale.y = 0.5
    marker.scale.z = 0.5
    # cambia colore
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    robotpub.publish(marker)

    rospy.loginfo("Spawned in Rviz and published")


def spawn_boxes(box):
    global boxpub
    boxpub = rospy.Publisher('/box_marker',MarkerArray, queue_size=10)
    boxes = MarkerArray()

    for i, (x, y, color) in enumerate(zip(box.x, box.y, box.colors)):
        marker = Marker()
        # Cambiato frame_id da "world" a "map"
        marker.header.frame_id = "map"
        marker.ns = "boxes"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

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

def init_boxes():
    pub = rospy.Publisher('/box_status', BoxInfo, queue_size=10)
    rospy.sleep(5)

    rospy.loginfo("Prima del while")
    box = BoxInfo()
    box.colors =  ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    box.x = [1, 2, 3, 0, 0 ,0]
    box.y = [1, 1, 1, 1, 2 ,3]
    box.status = [0, 0, 0, 0, 0 ,0]

    rospy.loginfo(f"box value: {box}")

    spawn_boxes(box)
    spawn_robot()

    #flag used : the topic is published only one time

    spawn = False
    while not rospy.is_shutdown():
        if not spawn:
            # box_status is published only one time without rate
            rospy.loginfo("Publishing into /box_status topic")
            pub.publish(box)
            rospy.loginfo("Published into /box_status topic")
        
            spawn = True
        else:
            break

def handle_reset_boxes(req):
    init_boxes()
    rospy.loginfo("Reset service call arrived, done = true")
    return reset_boxesResponse(done=True)


if __name__ == "__main__":

    rospy.init_node("init_spawn", anonymous=False)
    rospy.loginfo("Initializing system")
    # spawn_robot_boxes() 
    robotpub = rospy.Publisher('/robot_marker', Marker, queue_size=10)
    boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)
    
    rospy.Service('reset_boxes', reset_boxes, handle_reset_boxes)
    rospy.loginfo("Ready to reset boxes")

    init_boxes()

    rospy.spin()

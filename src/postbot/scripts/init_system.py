#!/usr/bin/python3
import rospy
import random
from postbot.msg import BoxInfo
from postbot.srv import reset_boxes, reset_boxesResponse
from visualization_msgs.msg import Marker, MarkerArray

boxpub = None

def spawn_boxes(box):
    global boxpub
    boxes = MarkerArray()

    for i, (x, y, color) in enumerate(zip(box.x, box.y, box.colors)):
        marker = Marker()
        marker.header.frame_id = "world"  # 'world' frame per Turtlesim
        marker.ns = "boxes"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0  # Orientamento neutro
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

    box = BoxInfo()
    box.colors =  ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    # Posizioni iniziali di esempio, personalizzare se necessario
    # Assicurarsi che le posizioni siano all'interno di [0, 11]
    box.x = [2, 2, 2, 8, 8, 8]
    box.y = [2, 4, 6, 2, 4, 6]
    box.status = [0, 0, 0, 0, 0, 0]

    spawn_boxes(box)

    spawn = False
    while not rospy.is_shutdown():
        if not spawn:
            pub.publish(box)
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
    boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)
    rospy.Service('reset_boxes', reset_boxes, handle_reset_boxes)
    rospy.loginfo("Ready to reset boxes")
    init_boxes()
    rospy.spin()

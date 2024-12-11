#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo
from visualization_msgs.msg import MarkerArray, Marker

boxpub = None

def init_boxes():
    global boxpub
    rospy.init_node("init_system", anonymous=False)
    boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)

    # Genera scatole casuali
    box_info = BoxInfo()
    box_info.colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    box_info.x = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    box_info.y = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    box_info.status = [0, 0, 0, 0, 0, 0]

    # Crea un MarkerArray per RViz
    marker_array = MarkerArray()
    for i, color in enumerate(box_info.colors):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.id = i
        marker.ns = "boxes"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = box_info.x[i]
        marker.pose.position.y = box_info.y[i]
        marker.pose.position.z = 0.25
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        if color == 'red':
            marker.color.r = 1.0
        elif color == 'blue':
            marker.color.b = 1.0
        # (Aggiungi tutti i colori)
        marker.color.a = 1.0
        marker_array.markers.append(marker)

    # Pubblica scatole
    boxpub.publish(marker_array)
    rospy.spin()

if __name__ == "__main__":
    init_boxes()

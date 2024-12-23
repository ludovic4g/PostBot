#!/usr/bin/python3
import rospy
from postbot.msg import BoxInfo
from postbot.srv import reset_boxes, reset_boxesResponse
from visualization_msgs.msg import Marker, MarkerArray

def spawn_boxes(box):
    global boxpub
    boxes = MarkerArray()

    for i, (x, y, color) in enumerate(zip(box.x, box.y, box.colors)):
        marker = Marker()
        marker.header.frame_id = "world"  # Frame where TurtleSim operates
        marker.header.stamp = rospy.Time.now()
        marker.ns = "boxes"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0 
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

def main():
    pub = rospy.Publisher('/box_status', BoxInfo, queue_size=10)
    rospy.sleep(1) 

    box = BoxInfo()
    box.colors =  ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    box.x = [2, 2, 2, 6, 6, 6]
    box.y = [2, 4, 6, 2, 4, 6]
    box.status = [0, 0, 0, 0, 0, 0]

    spawn_boxes(box)

    # Publish the initial box status
    rospy.sleep(1)
    pub.publish(box)

def handle_reset_boxes(req):
    main()
    rospy.loginfo("Reset service call arrived, done = true")
    return reset_boxesResponse(done=True)

if __name__ == "__main__":
    rospy.init_node("init_spawn", anonymous=False)
    rospy.loginfo("Initializing system")
    boxpub = rospy.Publisher('/box_marker', MarkerArray, queue_size=10)
    rospy.Service('reset_boxes', reset_boxes, handle_reset_boxes)
    rospy.loginfo("Ready to reset boxes")
    main()
    rospy.spin()

#!/usr/bin/python3
import rospy
from postbot.msg import MarbleInfo, BoxInfo
from postbot.srv import spawn_marble
from visualization_msgs.msg import Marker
import random

def marble_publisher(marble):
    marker = Marker()
    marker.header.frame_id = "world"  # 'world' frame per Turtlesim
    marker.header.stamp = rospy.Time.now()
    marker.id = random.randint(1, 1000)  # Unique ID per marble
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = marble.x
    marker.pose.position.y = marble.y
    marker.pose.position.z = 0.1
    marker.pose.orientation.w = 1.0  # Orientamento neutro
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    if marble.color == 'red':
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
    elif marble.color == 'blue':
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
    elif marble.color == 'green':
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
    elif marble.color == 'white':
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
    elif marble.color == 'purple':
        marker.color.r = 0.5
        marker.color.g = 0.0
        marker.color.b = 0.5
    elif marble.color == 'yellow':
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

    marker.color.a = 1.0
    marblepub.publish(marker)
    rospy.loginfo(f"Spawned Marble ({marble.x}, {marble.y}) {marble.color}")

def marble_callback(data):
    global spawn, color_position
    empty_boxes = [i for i, status in enumerate(data.status) if status < 1]
    if empty_boxes:
        color_position = random.choice(empty_boxes)
        spawn = True

def main():
    global marblepub, spawn, color_position
    rospy.init_node("spawn_marble_node", anonymous=False)
    marblepub = rospy.Publisher('/marble_marker', Marker, queue_size=10)
    pub = rospy.Publisher('/current_marble', MarbleInfo, queue_size=10)
    rospy.wait_for_service('/spawn_marble')
    spawn_marble_service = rospy.ServiceProxy('/spawn_marble', spawn_marble)

    colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    rospy.Subscriber("/box_status", BoxInfo, marble_callback)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if spawn:
            x = random.uniform(1, 10)  # All marbles within [1,10] for Turtlesim
            y = random.uniform(1, 10)
            color = colors[color_position]
            try:
                response = spawn_marble_service(x, y, color)
                if response.done:
                    new_marble = MarbleInfo()
                    new_marble.color = color
                    new_marble.x = x
                    new_marble.y = y
                    marble_publisher(new_marble)
                    pub.publish(new_marble)
                    spawn = False
                    color_position = 0
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        rate.sleep()

if __name__ == "__main__":
    main()

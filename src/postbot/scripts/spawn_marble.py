#!/usr/bin/python3
import rospy
from postbot.msg import MarbleInfo, BoxInfo
from postbot.srv import spawn_marble
from visualization_msgs.msg import Marker
import random

color_position = 0
spawn = False

def marble_publisher(marble):
    marblepub = rospy.Publisher('/marble_marker', Marker, queue_size=10)

    marker = Marker()
    marker.header.frame_id="world"
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = marble.x
    marker.pose.position.y = marble.y
    marker.pose.position.z = 0.1
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
    

    marblepub.publish(marker)
    rospy.loginfo(f"Spawned Marble with position ({marble.x}, {marble.y}) and color {marble.color}")
        

rospy.init_node("spawn_marble_node", anonymous=False)

# Publish to topic /current marble the color and position of the marble
rospy.loginfo("Waiting for /spawn_marble")
pub = rospy.Publisher('/current_marble', MarbleInfo, queue_size=10)
rospy.wait_for_service('/spawn_marble')
spawn_marble_service = rospy.ServiceProxy('/spawn_marble', spawn_marble)
rospy.loginfo("Post spawn_marble.")

colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']


def callback(data):
    global color_position, spawn
    rospy.loginfo("Sono nel callback")
    # Function to see which color is still not chosen
    empty_boxes = [index for index, value in enumerate(data.status) if value == 0]

    rospy.loginfo(f"boxes status: {empty_boxes}")
    if empty_boxes:
        color_position = random.choice(empty_boxes)
        spawn = True
        rospy.loginfo(f"color position chosen in callback {color_position}")




# Subscribe to topic /box status to see which color the marble has to be
rospy.Subscriber("/box_status", BoxInfo, callback)
#rospy.sleep(10)
# rospy.loginfo(f"color value: {chosen_color}")
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if spawn and color_position is not None:
        x = random.uniform(0, 5)
        y = random.uniform(0, 5)
        response = spawn_marble_service(x, y, colors[color_position])
        #rospy.loginfo(f"response value: {response.done}")
        if response.done:
            new_marble = MarbleInfo()
            new_marble.color = colors[color_position]
            new_marble.x = x
            new_marble.y = y
            marble_publisher(new_marble)
            rospy.loginfo(f"biglia: {new_marble.color}") 
            # CAMBIA COLORE SU RVIZ
            # MANDA IL ROBOT IN DIREZIONE DELLA PALLINA
            rospy.loginfo(f"Publishing into /current_marble topic")
            pub.publish(new_marble)
            rospy.loginfo(f"Published into /current_marble topic")
            #rospy.loginfo("publishd")
            spawn = False
            color_position = 0

        rate.sleep()
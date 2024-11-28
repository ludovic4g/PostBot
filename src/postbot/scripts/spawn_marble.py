#!/usr/bin/python3
import rospy
from postbot.msg import MarbleInfo, BoxInfo
from postbot.srv import spawn_marble
import random

color_position = 0
spawn = False

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
            rospy.loginfo(f"biglia: {new_marble.color}") 
            rospy.loginfo(f"Publishing into /current_marble topic")
            pub.publish(new_marble)
            rospy.loginfo(f"Published into /current_marble topic")
            #rospy.loginfo("publishd")
            spawn = False
            color_position = 0

        rate.sleep()
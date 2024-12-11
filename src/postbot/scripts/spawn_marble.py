#!/usr/bin/python3
import rospy
from postbot.srv import spawn_marble, spawn_marbleResponse
from visualization_msgs.msg import Marker
import random

# Publisher per il marker delle biglie
marblepub = None

def handle_spawn_marble(req):
    global marblepub

    # Genera posizione e colore della biglia
    x = random.uniform(0.5, 5.0)
    y = random.uniform(0.5, 5.0)
    colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    color = random.choice(colors)

    # Crea un marker per la biglia
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = 0
    marker.ns = "marble"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.1
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2

    # Imposta il colore della biglia
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
    elif color == 'yellow':
        marker.color.r = 1.0
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

    marker.color.a = 1.0

    # Pubblica la biglia
    marblepub.publish(marker)
    rospy.loginfo(f"Spawned marble at ({x}, {y}) with color {color}")

    # Risposta al servizio
    success = True
    return spawn_marbleResponse(done=success)

def spawn_marble_node():
    global marblepub
    rospy.init_node('spawn_marble', anonymous=False)

    # Inizializza il publisher per i marker
    marblepub = rospy.Publisher('/marble_marker', Marker, queue_size=10)

    # Servizio per spawnare biglie
    rospy.Service('spawn_marble', spawn_marble, handle_spawn_marble)
    rospy.loginfo("Ready to spawn a new marble")
    rospy.spin()

if __name__ == "__main__":
    spawn_marble_node()

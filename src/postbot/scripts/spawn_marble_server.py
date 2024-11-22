import rospy
from postbot.srv import spawn_marble, spawn_marbleResponse

def handle_spawn_marble(req):
    #only the result of the services needs to be sent
    success= True

    return spawn_marbleResponse(done=success)

def spawn_marble_server():
    rospy.init_node('spawn_marble_server')
    rospy.Service('spawn_marble', spawn_marble, handle_spawn_marble)
    rospy.loginfo("Ready to spawn a new marble")
    rospy.spin()


if __name__ == "__main__":
    
    spawn_marble_server()
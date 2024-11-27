#!/usr/bin/python3
import rospy
import random
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel
from postbot.msg import BoxInfo


def spawn_robot():
    initial_pose = rospy.get_param('robot_initial_pose')
    rospy.loginfo("Detected parameters for robot initial pose")
    pose = Pose()
    pose.position.x = initial_pose['x']
    pose.position.y = initial_pose['y']
    pose.orientation.z = 0.0

    # rospy.loginfo("Before gazebo service")
    # rospy.wait_for_service('/gazebo/spawn_sdf_model')
    # try:
    #    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    #   spawn_model(model_name="robot", model_xml="src/postbot/urdf/postbot.urdf", robot_namespace="", initial_pose=pose, reference_frame="world")
    #   rospy.loginfo("Robot spawned")
    # except rospy.ServiceException:
    #   rospy.loginfo("Robot NOT spawned")


def spawn_boxes():

    box_position = [
        {"x" : 1.0, "y" : 1.0, "z" : 0.0},
        {"x" : 2.0, "y" : 1.0, "z" : 0.0},
        {"x" : 3.0, "y" : 1.0, "z" : 0.0},
        {"x" : 1.0, "y" : 2.0, "z" : 0.0},
        {"x" : 2.0, "y" : 2.0, "z" : 0.0},
        {"x" : 3.0, "y" : 2.0, "z" : 0.0},
    ]

    colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']

    #rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        #spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        for i, pos in enumerate(box_position):
            pose = BoxInfo()
            pose.x = pos["x"]
            pose.y = pos["y"]
            pose.colors = colors
            pose.status = [0, 0 ,0 ,0 ,0 ,0]

            color = colors[i % len(colors)]

            model_name = f"box_{i}"
            model_path = f"src/postbot/urdf/box_{color}.urdf"

            #spawn_model(model_name=model_name, model_xml=model_path, robot_namespace="", initial_pose=pose, reference_name="world")
            #rospy.loginfo("Box spawned")

    except rospy.ServiceException:
        pass


def spawn_robot_boxes():

    spawn_robot()

    spawn_boxes()


def init_boxes():
    pub = rospy.Publisher('/box_status', BoxInfo, queue_size=10)
    rospy.sleep(5)

    rospy.loginfo("Prima del while")
    box = BoxInfo()
    box.colors =  ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    box.x = [0, 0, 0, 0, 0 ,0]
    box.y = [0, 0, 0, 0, 0 ,0]
    box.status = [0, 0, 0, 0, 0 ,0]

    rospy.loginfo(f"box value: {box}")

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
        




if __name__ == "__main__":

    rospy.init_node("init_spawn", anonymous=False)
    rospy.loginfo("Before spawning call")
    # spawn_robot_boxes()

    init_boxes()

    rospy.spin()

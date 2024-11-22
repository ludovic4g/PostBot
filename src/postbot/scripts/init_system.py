import rospy
import random
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnModel

def spawn_robot():
    initial_pose = rospy.get_param('robot_initial_pose')
    pose = Pose()
    pose.position.x = initial_pose['x']
    pose.position.y = initial_pose['y']
    pose.orientation.z = 0.0

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model(model_name="robot", model_xml="src/postbot/urdf/postbot.urdf", robot_namespace="", initial_pose=pose, reference_frame="world")
    except rospy.ServiceException:
        pass


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

    rospy.wait_for_service('/gazebo/spawn_sdf_model')

    try:
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        for i, pos in enumerate(box_position):
            pose = Pose()
            pose.position.x = pos["x"]
            pose.position.y = pos["y"]
            pose.position.z = pos["z"]

            color = colors[i % len(colors)]

            model_name = f"box_{i}"
            model_path = f"src/postbot/urdf/box_{color}.urdf"

            spawn_model(model_name=model_name, model_xml=model_path, robot_namespace="", initial_pose=pose, reference_name="world")

    except rospy.ServiceException:
        pass


def spawn_robot_boxes():
    rospy.init_node("init_spawn")

    spawn_robot()

    spawn_boxes()



if __name__ == "__main__":
    spawn_robot_boxes()

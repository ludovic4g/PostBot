import rospy
from postbot.msg import BoxGoal, BoxInfo, MarbleInfo


def managing_boxes():

    rospy.init_node('managing_boxes', anonymous=False)
    new_box_color = None

    def callback(data):
        new_box_color = data.color
        
    # Subscribes to the topic /current marble to take the color and its position
    rospy.Subscriber('/current marble', MarbleInfo, callback)


    rate = rospy.Rate(1)
    # Publish the coordinates of the box that has to be delivered
    pub = rospy.Publisher('/box goal', BoxInfo, )


if __name__ == '__main__':
    managing_boxes()





import rospy
from postbot.msg import MarbleInfo, BoxInfo
from postbot.srv import spawn_marble
import random

def spawn_marble():
    rospy.init_node("spawn_marble_node")

    # Publish to topic /current marble the color and position of the marble
    pub = rospy.Publisher('/current marble', MarbleInfo, queue_size=10)
    rospy.wait_for_service('/spawn marble')
    spawn_marble_service = rospy.ServiceProxy('/spawn marble', spawn_marble)

    rate = rospy.Rate(1)

    colors = ['red', 'blue', 'green', 'yellow', 'white', 'purple']
    color_position = 0

    def callback(data):
        global color_position
        empty_boxes = [index for index, value in enumerate(data.status) if value == 0]
        if empty_boxes:
            color_position = random.choice(empty_boxes)

    chosen_color = colors[color_position]

    # Subscribe to topic /box status to see which color the marble has to be
    rospy.Subscriber("/box status", BoxInfo, callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        x = random.uniform(0, 5)
        y = random.uniform(0, 5)

        response = spawn_marble_service(x, y, chosen_color)
        if response.done:
            new_marble = MarbleInfo()
            new_marble.color = chosen_color
            new_marble.x = x
            new_marble.y = y
            pub.publish(new_marble)

        rate.sleep()


if __name__ == '__main':
    try:
        spawn_marble()
    except rospy.ROSInterruptException:
        pass

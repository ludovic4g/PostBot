# PostBot🤖 - Delivery is on its way!

---

## What is PostBot?🤖

**PostBot** is an autonomous postman-robot that collects colored marbles and has to deliver them in boxes according to the same color. Everything is simulated thanks to **ROS** and **Gazebo** worlds to see exactly how the bot does its job!

The simulated world spawns marbles in different points of the map at every cycle, and this position is communicated to the bot through personalized messages. Once the message arrives, the bot goes to the communicated marble position to "collect" it and delivers it to the color-corresponding box. When all the boxes are full, the system resets the world by spawning new boxes in new positions, and the robot is repositioned at its initial starting point. The bot is now ready for new deliveries!

---

## Requirements👾

- **ROS**: For managing nodes, topics, custom messages, services, and parameters.
- **Gazebo**: For simulating the world by spawning marbles and boxes randomly.
- **RVIZ**: For monitoring in real-time navigation and robot status.
- **ROS Navigation Stack**: For autonomous movement and path planning.
- **TurtleBot3**: The autonomous robot responsible for marble deliveries.

---

## Core Components🕹️
### Nodes
- `Managing Deliveries Node`: For managing deliveries by communicating the Box Goal.
- `Spawned Marble Node`: For managing the position of the newly-spawned marble and to communicate its position.
- `Navigation Node`: Receive the delivery goal and plans the path the robot has to follow through **ROS Navigation Stack**. Once it arrives to destination it updates the box status.

### Topics
- `/current marble`: Communicates position and color of the newly-spawned marble.
- `/box status`: Communicates position, color and status of the boxes.
- `/box goal`: Communicates coordinates of the box goal.

### Custom Messages
- `MarbleInfo.msg`: Info about color and position of the spawned marble.
- `BoxStatus.msg`: Info about status, color and box coordinates.
- `BoxGoal.msg`: Coordinates of box goal.

### Services
- `/spawn marble`: Generates a new marble.
- `/reset boxes`: Reset boxed and spawn them in new random positions.

  
### Parameters
- `robot_configuration`: Movement settings of the robot.
- `robot_initial_pose`: Position of the robot when the system starts.

---

## How PostBot Works?🖲️

   - During the system startup, the robot is spawned at an initial position defined by the parameter `robot_initial_pose` and boxes have initial position defined randomly.
   - The nodes are initialized, and the system gets ready to spawn marbles and boxes.
   - The `Spawned Marble Node` uses the `/spawn marble` service to generate a new marble in a random position.
   - The position and color of the marble are published on the `/current marble` topic.
   - The `Managing Deliveries Node` receives the marble's information and determines the corresponding box for delivery.
   - It retrieves the box coordinates and publishes them on the `/box goal` topic.
   - The `Navigation Node` receives the goal coordinates from `/box goal` and uses the **ROS Navigation Stack** to plan a path.
   - The robot moves autonomously to the marble position, "collects" it, and then navigates to the box.
   - Once the marble is delivered, the `Navigation Node` updates the box's status on the `/box status` topic.
   - When all boxes are full:
     - The `Navigation Node` activates the `/reset boxes` service, resetting the boxes' positions.
     - The robot is reset to its predefined `robot_initial_pose`, restarting the delivery cycle.

---

## How To Install
(Coming soon!)

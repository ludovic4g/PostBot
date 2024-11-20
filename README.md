# PostBot🤖 - Delivery is on its way!

---

## What is PostBot?🤖

**PostBot** is an autonomous postman-robot that collects colored marbles and has to deliver them in boxes according to the same color. Everything is simulated thanks to **ROS** and **Gazebo** worlds to see exactly how the bot does its job!

The simulated world spawn marbles in different points of the map at every cicle and this position is communicated to the bot through personalized messages. Once the message arrives, the bot goes in the communicated marble position to "collect" it and delivers it to the color-corresponding box. When all the boxes are full, the system reset the world by spawning new boxes in new positions. The bot is now ready for new deliveries!

---

## Requirements👾

- **ROS**: For managing nodes, topics, custom messages, services and parameters.
- **Gazebo**: For simulating the world by spawning randomly marbles and boxes.
- **RVIZ**: For monitoring in real-time navigation and robot status.
- **ROS Navigation Stack**: For autonomous movement.
- **TurtleBot3**: The autonomous robot that has to deliver marbles.

---

## Core Components🕹️

### Nodes
- `Managing Deliveries Node`: For managing deliveries by communicating the `Box Goal` and updating the boxes status.
- `Spawned Marble Node`: For managing the position of the newly-spawned marble and to communicate its position.
- `Navigation Node`: Receive the delivery goal and plans the path the robot has to follow through **ROS Navigation Stack**.

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
- `Robot Settings`.
- `Inital Coordinates of the boxes`.

---

## How PostBot works?🖲️

1. The marble is spwaned thanks to `/spawn marble` service and the bot receives all info through `/current marble`.  
2. Now that it knows the box it has to deliver, it moves towards the coordinates taken from `/box goal`.  
3. It delivers the marble in the color-assigned box and updates the box status on `/box status`.  
4. When all the boxes are full, it activates `/reset boxes` and the cycle begins again.

---

## How To Install
(ancora da completare!)

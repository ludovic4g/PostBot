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

### Parameters
- `first_box_position`: Predefined positions for the boxes when the system starts.
- `robot_configuration`: Predefined initial position and movement settings of the robot.

---

## How PostBot Works?🖲️

**Initialization**:
   - During the system startup, the robot is spawned at an initial position defined by the parameter `robot_initial_pose`.
   - The nodes are initialized, and the system gets ready to spawn marbles and boxes.

**Marble Spawning**:
   - The `Spawned Marble Node` uses the `/spawn marble` service to generate a new marble in a random position.
   - The position and color of the marble are published on the `/current marble` topic.

**Delivery Planning**:
   - The `Managing Deliveries Node` receives the marble's information and determines the corresponding box for delivery.
   - It retrieves the box coordinates and publishes them on the `/box goal` topic.

**Robot Navigation**:
   - The `Navigation Node` receives the goal coordinates from `/box goal` and uses the **ROS Navigation Stack** to plan a path.
   - The robot moves autonomously to the marble position, "collects" it, and then navigates to the box.

**Delivery Completion**:
   - Once the marble is delivered, the `Managing Deliveries Node` updates the box's status on the `/box status` topic.

**Cycle Reset**:
   When all boxes are full:
     - The `Managing Deliveries Node` activates the `/reset boxes` service, resetting the boxes' positions.
     - The robot is reset to its predefined `robot_initial_pose`, restarting the delivery cycle.

---

## How To Install
(Coming soon!)

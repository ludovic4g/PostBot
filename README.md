# PostBot🤖 - Delivery is on its way!

---

## What is PostBot?🤖

**PostBot** is an autonomous postman-robot that collects colored marbles and has to deliver them in boxes according to the same color.
By using **Turtlesim** for motion simulation and **RViz** for visualization, the simulated world spawns marbles in different points of the map at every cycle, and this position is communicated to the bot through personalized messages. Once the message arrives, the bot goes to the communicated marble position to "collect" it and delivers to the color-corresponding box. When all the boxes are full, the system resets the world by emptying the boxes and spawning again new marbles. The bot is now ready for new deliveries!

---

## Requirements👾

- **Operating System:** Ubuntu 20.04
- **ROS Distribution:** ROS Noetic Ninjemys
- **Dependencies:**
  - `turtlesim`
  - `rviz`
  - `ros-<distro>-visualization-msgs`
  - `ros-<distro>-geometry-msgs`
  - `ros-<distro>-std-msgs`
  - `ros-<distro>-message-generation`
  - `ros-<distro>-message-runtime`

---

## Core Components🕹️
A rapid overwview of core components that PostBot uses:

### Nodes
- **Initialization Node** (`init_system.py`): Initializes the system by loading parameters and setting up necessary publishers and subscribers.
- **Managing Deliveries** (`managing_boxes.py`): For managing deliveries by communicating the Box Goal.
- **Spawning Marble** (`spawn_marble.py`): For managing the position of the newly-spawned marble and to communicate its position.
- **Navigation Node** (`navigation_node.py`): Receive the delivery goal and plans the path the TurteSim has to follow.. Once it arrives to destination it updates the box status. Subscribes to `/turtle1/pose` to receive the robot's current pose and publishes to `/turtle1/cmd_vel` to control movement.

### Topics

- **Custom Topics**
  - **`/current_marble`**
    - **Type:** `MarbleInfo`
    - **Description:** Publishes information about the current target marble, including its color and position. The `navigation_node` subscribes to this topic to determine the next destination for the robot.

- **`/robot_marker`**
    - **Type:** `visualization_msgs/Marker`
    - **Description:** Publishes a marker representing the robot's position and state in RViz. This allows for visual tracking and status indication within the simulation environment.

- **`/box_marker`**
    - **Type:** `visualization_msgs/MarkerArray`
    - **Description:** Publishes markers representing the boxes in the environment. Each box is visualized as a separate marker in RViz, displaying its color and occupancy status.

- **`/box_status`**
    - **Type:** `BoxInfo`
    - **Description:** Publishes the status of each box, including its color, position, and whether it is currently occupied. This information is used by nodes to manage interactions with boxes.

- **Built-In Topics**
    - **`/turtle1/pose`**
      - **Type:** `turtlesim/Pose`
      - **Description:** Publishes the current pose (position and orientation) of the turtle. The `navigation_node` subscribes to this topic to receive real-time updates of the robot's location.
  
  - **`/turtle1/cmd_vel`**
      - **Type:** `geometry_msgs/Twist`
      - **Description:** Accepts velocity commands to control the movement of the turtle. The `navigation_node` publishes to this topic to drive the robot towards targets.
  

### Custom Messages
- **Custom Messages:**
  - `BoxInfo.msg`:
    ```msg
    string[] colors
    float32[] x
    float32[] y
    int32[] status
    ```
  - `MarbleInfo.msg`:
    ```msg
    string color
    float32 x
    float32 y
    ```

- **Standard Messages:**
  - `geometry_msgs/Pose`: Represents the position and orientation of the robot.
  - `geometry_msgs/Twist`: Represents velocity commands.
  - `visualization_msgs/Marker`: Used for visualizing objects in RViz.


### Services
- **Custom Services:**
  - `reset_boxes.srv`:
    ```srv
    ---
    bool done
    ```

- **Built-in Services:**
  - `/turtle1/set_pose`: Sets the robot's pose in Turtlesim.

  
### Parameters
- **Configuration File (`robot_par.yml`):**
  ```yaml
  robot_initial_pose:
    x: 4.0
    y: 1.0
---

## How PostBot Works?🖲️
  - The `init_system.py` node is responsible for loading essential parameters from the `robot_par.yml` file. This includes setting the robot's initial position and orientation, as well as defining the locations of the boxes within the simulated environment.
  - After loading the parameters, `init_system.py` publishes visualization markers for both the robot and the boxes to RViz. These markers provide a graphical representation of the robot's starting point and the positions of the boxes, ensuring that the simulation environment is accurately set up for visualization.
  - The `spawn_marble.py` node manages the creation of marbles within the simulation. Depending on the configuration, marbles can be spawned at predefined locations or at random positions within the environment. These marbles serve as targets for the robot to collect and deliver to the designated boxes.
  - The `navigation_node.py` node plays a crucial role in controlling the robot's movement. It subscribes to the `/turtle1/pose` topic to receive real-time updates about the robot's current position and orientation.
  - Additionally, it listens to the `/current_marble` topic to obtain information about the marble that the robot should target next.
  - Using this information, `navigation_node.py` calculates the optimal path to the marble and publishes velocity commands to the `/turtle1/cmd_vel` topic, directing the robot towards the marble.
  - As the robot moves, the node also updates the robot's marker in RViz to reflect its new position, ensuring that the visualization remains consistent with the robot's actual movement.
  - Upon reaching a marble, the robot "collects" it by changing its color. This visual change indicates that the robot has successfully acquired the marble.
  - The robot then proceeds to navigate towards the corresponding box. Once it arrives at the box's location, it places the marble inside.
  - After depositing the marble, the robot updates the box's status to reflect that it is now occupied and resets its own color to indicate readiness for the next task.
  - To maintain flexibility and allow for repeated simulations, PostBot includes services such as `reset_robot` and `reset_boxes`.
  - The `reset_robot` service repositions the robot to its initial pose as defined in the parameters, ensuring that the robot starts from a known state.
  - The `reset_boxes` service resets the status of all boxes, making them available for new marbles. This allows the simulation to be restarted or adjusted without manual intervention.
  - **RViz** is utilized for providing a real-time visual representation of the simulation. It displays the robot, marbles, boxes, and the environment grid, offering users a comprehensive view of the robot's interactions within the simulated space.
  - Visualization markers are continuously updated to reflect changes in the robot's position, the status of the boxes, and the presence of marbles, enabling effective monitoring and interaction within the simulation environment.

---

## How To Install
We assumes you are using **Ubuntu 20.04 (Focal Fossa)** with **ROS Noetic Ninjemys**. Ensure you have the necessary permissions to install software and modify system configurations.
If you haven't installed ROS Noetic yet, follow the official guide: [ROS Noetic Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

  -**Create Catkin Workspace for managing ROS projects**
  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/
  ```

  -**Clone the PostBot repository**
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/ludovic4g/PostBot.git
  ```

  -**Initialize and source the Workspace**
  ```bash
  catkin_make
  source devel/setup.sh
  ```

  -**Launch PostBot**
   ```bash
  roslaunch postbot system.launch
  ```
  
---

## License
This project is licensed under the [MIT License](LICENSE).

---

## Collaborators
- **Ludovica Genovese** - [ludovic4g](https://github.com/ludovic4g)
- **Manuel Sica**  - [manuelsica](https://github.com/manuelsica)




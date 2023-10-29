# ICP-Based Localization
In this project, your task is to develop an ICP-Based Localizer system compliant with the ROS environment.

Localization is the task of estimating the pose of a sensor given its current measurement and the map of the environment. In the _planar_ case, if we have a reasonable initial guess of the sensor pose, it is possible to solve a Least Squares problem to refine the latter (otherwise, a multimodal  distribution has to be estimated, see __Probabilistic Robotics__ exam!).

To keep it short, `we can try alining the measurement on the map using the initial guess provided by the user`

To complete the project, you have to develop a node that:
- Listens to a `nav_msgs/OccupancyGrid`
  - Populate a KD-tree with the world coordinates of the __occupied cells__
- Listen to `/initialpose` topic to get the __Initial Guess__
- Listen to `sensor_msgs/LaserScan` messages
  - Compute the transform between the current scan and the map using ICP

Along with the localizer, you will need to start a `map_server` to publish the world map and `stage_ros` to simulate the robot moving in the map. Refeer to the `Requirements` section to get more informations.

Your work will be to complete the `Localizer2D` class (located in `include/localizer2d.h` and `src/localizer2d.cpp`) and the main node (located in `bin/localizer_node.cpp`).

# Obstacle Registration
Before starting the localizer, the node should receive both the map and its metadata from ROS. We implemented for you a `Map` class (located in `include/map.h`) that already parse these informations for you.

It provides informations regarding the size (rows and columns) of the map and easy accessors.

An occupancy grid, assigns a value at every cell of the map, namely:
  - **Occupied** : the cell represent a wall or a mapped object.
  - **Free** : the cell represents a free, travearsable area.
  - **Unknown** : the cell represents unknown or unmapped area.

These values are enumerated as a `CellType` (located in `include/map.h`).

- When an occupancy grid is received by your node, initialize the global _Map_ object, and pass it to the localizer.

- Complete the `Localizer2D::setMap` method, which explores the map and register all obstacles into a KD-Tree.

# Initial Pose
Using `RViz`, you can initialize the localizer by setting an initial estimate for the localizer. To do so, subscribe to the `/initialpose` topic and update the localizer pose based on the messages received (`Localizer2D::setInitialPose`)

# Localization
Once a scan and a sufficiently close initial pose estimate is given, the localizer can process the data and attempt localization (`Localizer2D::process`):
- Generate a scan prediction taken at the initial pose (`Localizer2D::getPrediction`)
- Set the ICP solver initial guess equal to the initial pose.
- Run ICP between the input scan and the prediction scan.
- Update the current pose of the laser based on the solver solution.

To generate the prediction, remember to also load range and angular parameters of the input scan to the localizer (`Localizer2D::setLaserParams`).

After the input scan is processed, remember to send a TF message (`geometry_msgs::TransformStamped`) using the `tf2_ros::TransformBroadcaster` object, along with a `nav_msgs::Odometry` message in the `/odom_out` topic.

You can test your localizer using `RViz`. We have provided a configuration that you can directly run after your node has started.

Go to the project directory, source ros and launch rviz with our configuration
```sh
rviz -d test_data/rviz.rviz
```

## Requirements


### Map Server
- Install the `ros-${DISTRO}-map-server` package. In our case (valid for Lattinone VM) we are using _ROS Noetic_
   ```sh
    sudo apt install ros-noetic-map-server
   ```
- To launch the node, go to the project directory, source ros and launch the `map_server` node
  - ```sh
    source /opt/ros/noetic/setup.bash
    rosrun map_server map_server test_data/cappero_map.yaml    
    ```

### Stage-ROS
- Install the `ros-${DISTRO}-stage-ros` and `ros-${DISTRO}-teleop-twist-keyboard` package. As above, we are using _ROS Noetic_
  ```sh
  sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
  ```
- To launch the simulator, go to the project directory, source ros, launch the roscore and launch the `stageros` node
  - ```sh
    rosrun stage_ros stageros test_data/cappero.world
    ```

- You should see the simulator running on a dedicated window. The interfaces are ROS-based and the topics of interest for us are:
  - `/base_scan` : Contains laser scanner data from the robot (Output)
  - `/cmdvel` : Robot commands (Input)
  
## General TIPS
- If you have doubts on how to link your package in the ROS environment, take a look at the `rp_10_scan_matcher` exercise and take a look at its README.

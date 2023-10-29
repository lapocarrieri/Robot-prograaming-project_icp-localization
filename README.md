# Robot-programming-project_icp-localization
Icp localizer using Ros system

## The task
In this project, the task is to develop an ICP-Based Localizer system compliant with the ROS environment.

Localization is the task of estimating the pose of a sensor given its current measurement and the map of the environment. In the _planar_ case, if we have a reasonable initial guess of the sensor pose, it is possible to solve a Least Squares problem to refine the latter (otherwise, a multimodal  distribution has to be estimated, see __Probabilistic Robotics__ exam!).

To keep it short, `we can try alining the measurement on the map using the initial guess provided by the user`

## Description

Once a scan and a sufficiently close initial pose estimate is given, the localizer can process the data and attempt localization (`Localizer2D::process`):
- Generate a scan prediction taken at the initial pose (`Localizer2D::getPrediction`)
- Set the ICP solver initial guess equal to the initial pose.
- Run ICP between the input scan and the prediction scan.
- Update the current pose of the laser based on the solver solution.


## Running the Code

Follow the steps below to run the code:

### 1. Download ROS Dependencies

Make sure you have ROS installed on your system. If you haven't installed it yet, please refer to the ROS installation documentation (http://wiki.ros.org/ROS/Installation) for guidance.

### 2. Create a Project Folder

Create a directory to store your project. You can do this using the following command:

```
mkdir -p /path/to/your/project/folder
```
Replace /path/to/your/project/folder with the actual path where you want to create your project folder.
### 3. Clone the Repository
Navigate to your project folder and clone this GitHub repository into it using Git:
```
cd /path/to/your/project/folder
git clone https://github.com/yourusername/icp_localization.git
```
### 4. Requirements
Navigate to your ROS workspace and source the devel/setup.bash file:
```
source /path/to/your/ros/workspace/devel/setup.bash
roscore
```
### 5. Run the Localization Node
In a new terminal, source the code again, and then run the ICP localization node:
```
source /path/to/your/ros/workspace/devel/setup.bash
rosrun icp_localization localizer_node
```
### 6. Map Server
- Install the `ros-${DISTRO}-map-server` package. In our case (valid for Lattinone VM) we are using _ROS Noetic_
   ```sh
    sudo apt install ros-noetic-map-server
   ```
- To launch the node, go to the project directory, source ros and launch the `map_server` node
  ```sh
  cd /home/lattinone/catkin_ws/02_icp_localization
    source /opt/ros/noetic/setup.bash
    rosrun map_server map_server test_data/cappero_map.yaml    
    ```

### 7. Visualize with RViz
You can test your localizer using `RViz`. We have provided a configuration that you can directly run after your node has started.

Go to the project directory, source ros and launch rviz with our configuration
```
cd /path/to/your/project/folder/icp_localization
source /opt/ros/noetic/setup.bash
rviz -d test_data/rviz.rviz
```
### 8. Stage-ROS
- Install the `ros-${DISTRO}-stage-ros` and `ros-${DISTRO}-teleop-twist-keyboard` package. As above, we are using _ROS Noetic_
  ```sh
  sudo apt install ros-noetic-stage-ros ros-noetic-teleop-twist-keyboard
  ```
- To launch the simulator, go to the project directory, source ros, launch the roscore and launch the `stageros` node
   ```sh
  cd /home/lattinone/catkin_ws/02_icp_localization
   source /opt/ros/noetic/setup.bash
    rosrun stage_ros stageros test_data/cappero.world
    ```
You should now have the ICP localization system up and running within your ROS environment. If you encounter any issues or have questions, please refer to the project documentation or seek assistance from the project maintainers.






https://github.com/lapocarrieri/Robot-prograaming-project_icp-localization/assets/56505429/dcea8e4c-6144-4f96-beb5-5d4fa4de97e1




## Authors

Contributors names and contact info

Lapo Carrieri 



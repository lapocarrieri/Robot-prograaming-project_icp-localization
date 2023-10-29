# Robot-prograaming-project_icp-localization
Icp localizer using Ros system



## Description

An in-depth paragraph about your project and overview of use.

## Getting Started
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
### 4. Source the Code
Navigate to your ROS workspace and source the devel/setup.bash file:
```
source /path/to/your/ros/workspace/devel/setup.bash
roscore
```
### 6. Run the Localization Node
In a new terminal, source the code again, and then run the ICP localization node:
```
source /path/to/your/ros/workspace/devel/setup.bash
rosrun icp_localization localizer_node
```
### 7. Instantiate the Map
In another terminal, navigate to the project directory and source the ROS setup:
```
cd /path/to/your/project/folder/icp_localization
source /opt/ros/noetic/setup.bash
rosrun map_server map_server test_data/cappero_map.yaml
```
### 8. Visualize with RViz
In a new terminal, navigate to the project directory and source the ROS setup:
```
cd /path/to/your/project/folder/icp_localization
source /opt/ros/noetic/setup.bash
rviz -d test_data/rviz.rviz
```
### 9. Simulate Robot Movement
In a new terminal, navigate to the project directory and source the ROS setup:
```
cd /path/to/your/project/folder/icp_localization
source /opt/ros/noetic/setup.bash
rosrun stage_ros stageros test_data/cappero.world
```

You should now have the ICP localization system up and running within your ROS environment. If you encounter any issues or have questions, please refer to the project documentation or seek assistance from the project maintainers.






## Authors

Contributors names and contact info

ex. Dominique Pizzie  



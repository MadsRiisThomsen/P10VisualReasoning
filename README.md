# P10VisualResoning
This code is meant to run on the "ROS PC" as described in the AAU_franka_moveit repository: [LINK](https://github.com/HuchieWuchie/AAU_franka_moveit)

It first uses the P8 project on the franka (TODO: reduce the amount of modules used), then the visual reasoning is custom/new code used for the project

The P8 readmo doesnt mention it, but these modules need to be installed:
modules to install:
	websockets
	jsonpickle
	transformers
	torch

use pip3 install [module]

#Readme from P8
## Compiling the Catkin Workspace
There is a name conflict between the em and empy packages on python3, so run `pip3 uninstall em` and then `pip3 install empy`
Python 3 is used for the Python modules. To ensure that Catkin uses Python 3 for compiling the packages, use 
`catkin_make -DPYTHON_EXECUTABLE=[path to your python 3 executable]` in the root of the Catkin workspace.

## Running a node
If you have problems running a Python 3 node, make sure to source in the following order: ROS Melodic -> (Anaconda if you use that for python 3 - not sure this is required) -> workspace setup.bash.
If you get errors about the node not being an executable or something like that, add executable permission (`chmod +x [path_to_node]`). If you get Python errors when trying to run the node, make sure `rospkg` is installed for Python 3 using `pip install rospkg`. Both Melodic Python 2.7 and your Python 3 has to be in PYTHONPATH environment variable, since rospy and other modules are located without Melodic's Python 2.7.

### cv_bridge problem
If there is a problem with cv_bridge, it needs to be compiled for python 3. See [here](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674).

---

## Speech to text
For Azure Cognitive Services (what we are using now):
1. `pip install azure-cognitiveservices-speech`

## Running the code
Make sure that ROS, cv_bridge, UR-ROS and this workspace is sourced when running the code. It is very important that cv_bridge is BEFORE ROS in PYTHONPATH environment variable (ROS should preferably be the last).
1. `roslaunch dialog_flow hri_backbone_launch.launch azure_key:=[api key to Azure]`
2. `rosrun bin_picking moveit_interface_node.py` - IMPORTANT: Needs to be python 2!
3. `Run dialog_flow.py` NOTE: The import paths for the dialog_flow.py does not allow it to be run from terminal, without the correct PYTHONPATH set. 
To be able to run it without updating the import paths, source the ROS, cv_bridge and this workspace and start PyCharm and set all packages as "Sources root". Then run dialog_flow.py from PyCharm.

## Run from terminal
`export PYTHONPATH=${PYTHONPATH}:[path-to-catkin-ws]/src/grounding`

If you want to visualize the robot in RViz: `roslaunch ur5_moveit_config moveit_rviz.launch config:=true`

### Examples of controlling WeBots directly using ROS
These examples needs to have the controller in WeBots set to the `ros` controller.

__Get image from webots__
1. Subscribe to /camera/image and/or /range_finder/image
2. Publish std\_msgs/Bool = 1 to /publish_images


__Control gripper/suction__
Topics: /gripper/set_state
		/suction/set_state

Type: std_msgs/Bool

Value: 1 = close gripper/start suction
	   0 = open gripper/stop suction

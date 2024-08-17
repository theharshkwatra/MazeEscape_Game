# Maze Escape Game

This project is created using ROS Noetic and Gazebo.

Installation and Setup
Step 1: Create a ROS Workspace
Open a terminal and create a workspace:

bash
Copy code
mkdir -p ~/maze_escape_ws/src
cd ~/maze_escape_ws/src
Add the downloaded src file into the src directory of the workspace.

Step 2: Build the Workspace
Navigate to the workspace root:

bash
Copy code
cd ~/maze_escape_ws
Compile the workspace:

bash
Copy code
catkin_make
Step 3: Source the Setup File
Source the setup file to configure your environment:

bash
Copy code
source devel/setup.bash
Running the Simulation
Open a terminal and navigate to the workspace:

bash
Copy code
cd ~/maze_escape_ws
Source the setup file:

bash
Copy code
source devel/setup.bash
Launch the Gazebo simulation:

bash
Copy code
roslaunch maze_escape gazebo.launch
The simulation will run in Gazebo, and you can control the robot using the keys specified in the terminal.


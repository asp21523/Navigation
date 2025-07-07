Autonomous System Planning and Control for Mobile Robot Navigation
This ROS-based project implements a complete autonomous navigation system for a TurtleBot3 robot, integrating global path planning, local waypoint following, and obstacle avoidance using potential fields.

📁 Repository Structure

├── Autonomous_systems.pdf
├── CMakeLists.txt
├── launch
│   ├── csb.launch
│   ├── map1.launch
│   ├── map_world.launch
│   └── office.launch
├── map1.sdf
├── materials
│   ├── scripts
│   │   └── autonomous_lane.material
│   └── textures
│       └── autonomous sys.png
├── maze
├── models
│   └── lane_path
│       ├── materials
│       │   ├── scripts
│       │   │   └── Lane.material
│       │   └── textures
│       │       └── lane.jpeg
│       ├── meshes
│       ├── model.config
│       └── model.sdf
├── my_map.sdf
├── my_robot_planner
│   ├── CMakeLists.txt
│   ├── ~p
│   ├── package.xml
│   ├── scripts
│   │   ├── global_planner.py
│   │   ├── kinematic_controller.py
│   │   ├── navigator.py
│   │   ├── potential_fields.py
│   │   └── __pycache__
│   │       ├── global_planner.cpython-38.pyc
│   │       ├── kinematic_controller.cpython-38.pyc
│   │       └── potential_fields.cpython-38.pyc
│   └── src
├── package.xml
├── pranav.gitignore
├── pranav.mp4
└── README.md




🧠 System Overview
navigator.py: Integrates all the scripts, runs potentail fileds script when needed.

global_planner.py: Implements A* search for global path planning.

kinematic_controller.py: Unicycle model controller for velocity commands.

potential_fields.py: Reactive local planner using potential fields.

🧱 Dependencies
OS: Ubuntu 20.04

ROS: Noetic Ninjemys

Robot: TurtleBot3 (Install: ros-noetic-turtlebot3, ros-noetic-turtlebot3-simulations)

Python Packages: numpy, scipy, matplotlib, rospy, tf

Install ROS packages:
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations

🚀 How to Run
1.  Clone the repository
cd ~/catkin_ws/src
git clone https://github.com/asp21523/Navigation.git
cd ..
catkin_make

2. Launch the simulation

to launch the robot in the custom world: roslaunch asgn map1.launch
to get the map                         : rosrun map_server map_server ~/map1.yaml
to run the navigation node             : rosrun asgn navigator.py
to give inputs and visualize in rviz   : rviz
 and then add map and path in rviz, give inputs using 2D pose.

📄 Documentation
pranav.mp4             :for simulation video
Autonomous_systems.pdf : IEEE format report

👤 Author
Sai Pranav Adabala
Technische Hochschule Deggendorf
Dept. of Mechatronics and Cyber-Physical Systems
sai.adabala@stud.th-deg.de


 


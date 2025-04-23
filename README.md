# CDE2310 Group 9 – AY24/25 S2

**CDE2310** is a course offered at the National University of Singapore (NUS) under the Innovation and Design Programme (iDP). Commonly referred to as *Fundamentals of Systems Design*, it emphasizes the application of systems engineering principles to design and build autonomous robotic systems.

This repository showcases the source code for navigation and control for our system.

**Project Goal**: To design and build a robot capable of exploring a map autonomously while identifying heat signatures in the environment.

---

# Setup Instructions

These steps will help you get started with our autonomous navigation stack using ROS 2, `nav2`, and `slam_toolbox`.

## On Laptop

### 1. Install ROS 2 Humble

Follow the official installation instructions for your platform:  
[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)

### 2.  Install nav2 and slam_toolbox

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
```

### 3. Create a workspace directory

```bash
mkdir nav_ws
cd nav_ws
```

### 4. Create a ROS2 package with the package name "navigation_node"

```bash
mkdir src
cd src
ros2 pkg create --build-type ament_python navigation_node
```

### 5. Delete the "navigation_node" directory and clone the repository

```bash
rm -rf navigation_node
git clone https://github.com/manudagur87/CDE2310-Group9.git
cd CDE2310-Group9
rm -rf RPi
mv navigation_node ..
cd ..
rm -rf CDE2310-Group9
```
### 6. Bulid the package

```bash
cd ..
colcon build --packages-select navigation_node
source install/setup.bash
```
## On SBC

### 1. Setup turtlebot

Follow the instructions in 3.2 and 3.3 to set up your Turtlebot
[https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup)

### 2. Creating "rosbu" alias

This is to make it easier to run the Turtlebot bringup everytime.

```bash
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'alias rosbu='ros2 launch turtlebot3_bringup robot.launch.py'' >> ~/.bashrc
```
Now, anytime you want to run bringup, all you have to do is write the "rosbu" command in the terminal

### 3. Cloning the git repository

```bash
git clone https://github.com/manudagur87/CDE2310-Group9.git
rm -rf navigation_node
mv RPi ..
cd ..
rm -rf CDE2310-Group9
```

### 4. Installing required libraries

```bash
sudo apt update -y
sudo apt install -y i2c-tools   
sudo apt install -y libgpiod-dev python3-libgpiod 
pip3 install adafruit-blinka 
pip3 install adafruit-circuitpython-amg88xx 
pip3 install RPi.GPIO
```

# Running Instructions

## On SBC

Run: `rosbu`

Open a new terminal and run the following commands:

```bash
cd RPi
python3 launcher_heatsensor.py
```

## On Laptop

Open 4 terminals and run the following commands in each of them

`rviz2 -d turtlebot3_ws/src/turtlebot3/turtlebot3_cartographer/rviz/tb3_cartographer.rviz`

`ros2 launch navigation_node online_async_launch.py`

`ros2 launch navigation_node navigation_launch.py`

`ros2 run navigation_node nav`

# Acknowledgement

Special Thanks to:

- CDE2310 Instructors
    - Mr Chew Wanlong, Nicholas
    - Mr Ee Wei Han, Eugene
    - Mr Royston Shieh Teng Wei
    - Mr Soh Eng Keng
- CDE2310 TAs
- [SeanReg's Frontier Exploration](https://github.com/SeanReg/nav2_wavefront_frontier_exploration)

# Project Members:

- Manu Dagur
- Paterson Wong
- Tsai Ho Hsun
- Wang Jiawei
- Shreevrind Kajaria

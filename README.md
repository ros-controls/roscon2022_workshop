# ROSCon 2022 workshop on ros2_control

Thank you for your interest in ros2_control.
This repository was developed with the purpose of supporting the workshop on *ros2_control* @ ROSCon2022 in Kyoto, Japan.
Whether you participated or not, the repository will provide you with detailed instructions on how to use the *ros2_control* framework and explain functionality and purpose of its individual parts.

### Installing this repository

After cloning the repository, go to your source workspace and execute the following commands to import the necessary repositories and to install all dependencies:
```
vcs import --input roscon2022_workshop/roscon2022_workshop.repos .
rosdep update
rosdep install -y -i --from-paths .
```

## What is *ros2_control*

In short, ros2_control is a control framework for ROS2.
But actually, it is much more, it is the kernel of the ROS2 system that controls robots:
 - it abstracts hardware and low-level control for other frameworks such as MoveIt2 and Nav2;
 - it provides resource access management,
 - controls their lifecycle and so on.

For more details check [this presentation](https://control.ros.org/master/doc/resources/resources.html#ros-world-2021)


## Structure of this repository (and workshop)

The structure of the repository follows the flow of integrating robots with ROS2 and these are the following steps:

1. 📑 Setting up the hardware description for *ros2_control*

   1. 🗒 Setting up the robot's URDF using XACRO
   2. 📝 Extending the robot's URDF with the `<ros2_control>` tag

2. 🖥 Using the *Mock Hardware* plugin for easy and generic testing of the setup (and how it can save you ton of time and nerves)

   1. 🛠 How to setup *Mock Hardware* for a robot?
   2. 🔩 How to test it with an off-the-shelf controller?

3. ⚙ Getting to know the roles of the main components of the *ros2_control* framework: *Controller Manager*, *Controllers*, *Resource Manager* and *Hardware Interface*

4. 🔬 Introspection of the *ros2_control* system

5. 💻 Simulating your hardware using Gazebo Classic and Gazebo

6. 🔃 Become familiar with the lifecycle of controllers and hardware. Learn how to use them.

7. 🤖 How to write a hardware interface for a robot

8. 🛂 How to write a controller


## Details about the workshop

### 1. 📑 Setting up the hardware description for *ros2_control*

##### GOAL

  - learn how to setup the URDF of a robot using XACRO macros
  - learn what URDF changes are needed to integrate a robot with `ros2_control`


##### 🗒 Setting up URDF using XACRO for a robot

For any robot that is used with ROS/ROS2 an URDF description is needed.
This description holds information about kinematics, visualization (e.g., for Rviz2) and collision data.
This description is also used by popular ROS2 high-level libraries like, MoveIt2, Nav2 and Simulators.

In this excercise we will focus on setting up the description using the XACRO format which is highly configurable and parameterizable and generally better to use than the static URDF format.

##### Task

Branch: `1-robot-description/task`

Task is to setup the XACRO for RRbot in a package called `controlko_description`.

Kinematics:

  - 2 DoF
  - 1st joint is on a pedestal (box 30x30x30 cm) 30 cm above the ground and rotates around the axis orthogonal to the ground
  - 1st link is 50cm long (cylinder with 20cm diameter)
  - 2nd joint is rotation orthogonal to the first link's height
  - 2nd link is 60cm long  (10x10cm cross-section 5x5cm)

Hardware:

  - Force Torque Sensor at TCP (6D)
  - 2 digital inputs and output (outputs can be measured)

References:

  - https://wiki.ros.org/urdf
  - https://wiki.ros.org/urdf/XML

Files to create or adjust:

  - `rrbot_macro.xacro` - macro with kinematics and geometries for the `rrbot`
  - `rrbot.urdf.xacro` - main xacro file for the robot where macro is instantiated
  - `view_robot.launch.py` - loading and showing robot in `rviz2`


**TIPP**: `RosTeamWS` tool has some scripts that can help you to solve this task faster (on the branch is this already implemented). Resources:

  - [Commonly used robot-package structure](https://stoglrobotics.github.io/ros_team_workspace/master/guidelines/robot_package_structure.html)
  - [Creating a new package](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros_packages/create_package.html)
  - [Setting up robot description](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros_packages/setup_robot_description_package.html)


##### Solution:

Branch: `1-robot-description/solution`

Check the files listed above and execute:
```
ros2 launch controlko_description view_rrbot.launch.py
```
to view the robot and move its joints using the `Joint State Publisher` GUI.

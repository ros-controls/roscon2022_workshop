# ROSCon 2022 workshop on ros2_control

Thank you for your interest in ros2_control.
This repository is developed with a purpose of supporting workshop on *ros2_control* @ ROSCon2022 in Kyoto, Japan.
Regardless if you could or could not participate the repository will provide you with detailed instruction on how utilize *ros2_control* framework and explain functionality and purpose of its individual parts.

### Installing this repository

After cloning the repository, go to your source workspace and execute following command to install all dependencies:
```
vcs import --input roscon2022_workshop/roscon2022_workshop.repos .
rosdep update
rosdep install -y -i --from-paths .
```

## What is *ros2_control*

Simply, ros2_control is a control framework for ROS2.
But actually, it is much more, it is kernel of ROS2 system that controls robots:
 - it abstracts hardware and low-level control for other framework like MoveIt2 and Nav2;
 - it provide resource access management,
 - controls theirs lifecycle and so on.

For more details check [this presentation](https://control.ros.org/master/doc/resources/resources.html#ros-world-2021)


## Structure of this repository (and workshop)

The structure of the repository follows the flow of integrating robots with ROS2 and those are the following steps:

1. 📑 Setting up hardware description for *ros2_control*

   1. 🗒 Setting up URDF using XACRO for a robot
   2. 📝 Extending robot's URDF with `<ros2_control>` tag

2. 🖥 Using *Mock Hardware* plugin for simple and generic testing of the setup (and how it can save you ton of time and nerves)

   1. 🛠 How to setup *Mock Hardware* for a robot?
   2. 🔩 How to test it with a of-the-shelf controller?

3. ⚙ Getting know the roles of the main components of *ros2_control* framework: *Controller Manager*, *Controllers*, *Resource Manager* and *Hardware Interface* -- 30 min

4. 🔬 Introspection of *ros2_control* system

5. 💻 Simulating your hardware using Gazebo Classic and Gazebo

6. 🔃 Getting familiar with the lifecycle of controllers and hardware and how to use it

7. 🤖 How to write a hardware interface for a robot

8. 🛂 How to write a controller

9. 🔗 Reusing standard controller and creating controller-chains

10. ♻ Modular reuse of hardware drivers for complex systems

11. 🤖🤖🤖 Managing multiple robots with ros2_control

12. 👑 Dr. Denis' Tips:

    1. 💉 Parameter injection
    2. ⚖ Typical setup of robots and it packages


## Details about the workshop

### 1. 📑 Setting up hardware description for *ros2_control*

##### GOAL

  - learn how to setup URDF for a robot using XACRO macros
  - learn what URDF changes are needed to integrate a robot with `ros2_control`


##### 🗒 Setting up URDF using XACRO for a robot

For any robot that is used with ROS/ROS2 URDF description is needed.
This description holds information about kinematics, visualization (e.g., for Rviz2) and collision data.
This description is are also used by popular ROS2 high-level libraries like, MoveIt2, Nav2 and Simulators.

In this excercise we will focus on setting up the description using XACRO format which is highly configurable and parameterizable and generally better to use then static URDF format.

##### Task

Branch: `1-robot-description/task`

Task is to setup the XACRO for RRbot in a package called `controlko_description`.

Kinematics:

  - 2 DoF
  - 1st joint is on a podest (box 30x30x30 cm) 30 cm above the ground and rotates around the axis orthogonal to the ground
  - 1st link ist 50cm long (cylinder with 20cm diameter)
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
to view the robot and move its joins using `Joint State Publisher` GUI.


### 2. 🖥 Using *Mock Hardware* plugin for simple and generic testing of the setup

##### GOAL

  - learn what is *Mock Hardware* and how to use it
  - learn how you can fast and easily test you controller's setup and parameters before you deal with simulation or real hardware

*Mock Hardware* is mocking ros2_control `Hardware Interface` based on the robot's description from the `ros2_control` tag.
Its purpose is to simplify and boost development process when creating new controller or setting up their configuration.
Advantage of using it, over simulation or real hardware, is very fast start-up and lean functionality.
It is a well tested module helping you to focus on other components in your setup knowing that your "hardware" behaves ideally.
*NOTE:* the functionality of *Mock Hardware* is intentionally limited and it only enables you to reflect commanded values on the state interfaces with the same name. Nevertheless, this is sufficient for the most tasks.

**TIPP**:

  - Dr. Denis recommends you to always start develop things first with the Mock Hardware and then start switching to simulation or real hardware. This way you save you time dealing with broken setup with simulation or hardware in the loop.


##### 🛠 How to setup *Mock Hardware* for a robot?

1. Add `hardware` tag under the `ros2_control` tag with plugin `mock_components/GenericSystem` and set `mock_sensor_commands` parameter.
The parameter create fake command interface for sensor values than enables you to simulate the values on the sensor.

2. Create launch file named `rrbot.launch.py` that start ros2_control node and with the correct robot description.

**NOTE**: Currently, there is only `GenericSystem` mock component, which can mock also sensor or actuator components (because they just have reduced feature set compared to a system).

##### 🔩 How to test it with a off-the-shelf controller?

1. Setup the following controllers for the `RRBot`:

- `Joint State Broadcaster` - always needed to get `/joint_states` topic from a hardware.
- `Forward Command Controller` - sending position commands for the joints.
- `Joint Trajectory Controller` - interpolating trajectory between from the position command for the joint.

2. Add to launch file spawning (loading and activating) of controllers.

3. Test forward command controller by sending a reference to it using `ros2 topic pub` command

4. Create a launch file that start `ros2_controllers_test_nodes/publisher_joint_trajectory_controller` to publish goals for JTC.

**TIPP**: `RosTeamWS` tool has some scripts that can help you to solve this task faster. Resources:

  - [Commonly used robot-package structure](https://stoglrobotics.github.io/ros_team_workspace/master/guidelines/robot_package_structure.html)
  - [Creating a new package](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros_packages/create_package.html)
  - [Setting up robot bringup](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros_packages/setup_robot_bringup_package.html)

##### Solution:

Branch: `2-robot-mock-hardware`

Check the files listed above and execute:
```
ros2 launch controlko_bringup rrbot.launch.py
```
then publish command to the forward command controller:
```
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "
layout:
 dim: []
 data_offset: 0
data:
 - 0.7
 - 0.7"
```

To start `RRBot` with JTC execute:
```
ros2 launch controlko_bringup rrbot.launch.py robot_controller:=joint_trajectory_controller
```
and open new terminal and execute:
```
ros2 launch controlko_bringup test_joint_trajectory_controller.launch.py
```

**NOTE**: delay between spawning controllers is usually not necessary, but useful when starting a complex setup. Adjust this specifically for the specific use-case.


### 3. ⚙ Getting know the roles of the main components of *ros2_control* framework

Start the previous example one more time and try to answer the following questions:

1. What and where is *Controller Manager*?
2. What are *Controllers*? How they can be seen in the ROS2?
3. What is *Resource Manager*? Where can you see it? How to access it?
4. What is *Hardware Interface*? Where is this stored? How to interact with it?

##### Solution:

TODO: Add diagram/figure here.


### 4. 🔬 Introspection of *ros2_control* system

There are two options to interact with the ros2_control, first, using CLI interface with command `ros2 control <command>` (package `ros2controlcli`), and second, using services of controller manager directly.

Try figure out how to answer following questions using those tools:

1. What controllers are loaded in the system?
2. What is the state of controllers?
3. What hardware interfaces are in which state?
4. Which interfaces are available?
5. How can we switch between `forward_position_controller` and `joint_trajectory_controller`?
6. What happens when you try to run all controllers in parallel?
7. What interfaces are controllers using?

Also there are few graphical tools available for `ros2_control`: `rqt_controller_manager` and `rqt_joint_trajectory_controller`. Try to use those tools.

##### Solution:

Answers to the questions:

1. `ros2 control list_controllers`
2. `ros2 control list_controllers`
3. `ros2 service call /controller_manager/list_hardware_components controller_manager_msgs/srv/ListHardwareComponents {}`
4. `ros2 control list_hardware_interfaces`
5. `ros2 run controller_manager spawner forward_position_controller --inactive`
   `ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller`
6. See output in the terminal where `ros2_control_node` is running: `ros2 control switch_controllers --activate joint_trajectory_controller`
7. `ros2 control list_controllers -v`


### 5. 💻 Simulating your hardware using Gazebo Classic and Gazebo

ros2_control is integrated with simulators using simulator-specific plugins.
Those plugins extend controller manager with simulator-related functionalities and enables loading hardware interfaces that are created specifically for the simulator.
Simulators are using description under `<ros2_control>` to setup the interfaces.
They are searched for interfaces with standard names, `position`, `velocity` and `effort`, to wire them with the internal simulator-states.

The plugins and interface for the simulators are the following:

**Gazebo Classic**
  - Package: `gazebo_ros2_control`
  - Simulator plugin: `libgazebo_ros2_control.so`
  - HW interface plugin: `gazebo_ros2_control/GazeboSystem`

**Gazebo**
  - Package: `gz_ros2_control`
  - Simulator plugin: `libign_ros2_control-system.so`
  - HW interface plugin: `ign_ros2_control/IgnitionSystem`
  **NOTE** `ign` will be switched to `gz` very soon!


Let's define those plugins for `RRBot`:

1. Extend `rrbot.urdf.xacro` with `<gazebo>` tags defining simulator plugins and parameters.
2. Add hardware interface plugins under `<ros2_control>` tag in the file `rrbot_macro.ros2_control.xacro`.
3. Add new launch file `rrbot_sim_gazebo_class.launch.py` for starting Gazebo Classic simulation.
4. Add new launch file `rrobt_sim_gazebo.launch.py` for starting Gazebo simulation.

##### Solution:

Branch: `5-simulation`

Check updated files from the above list.
To start Gazebo Classic simulation use:
```
ros2 launch controlko_bringup rrbot_sim_gazebo_classic.launch.py
```

To start Gazebo simulation use:
```
ros2 launch controlko_bringup rrbot_sim_gazebo.launch.py
```

Now execute test script for joint trajectory controller to move the robot.
```
ros2 launch controlko_bringup test_joint_trajectory_controller.launch.py
```

**NOTE**: When running simulation be sure to set the joint limits defined in the macro file.


### 6. 🔃 Getting familiar with the lifecycle of controllers and hardware and how to use it

*ros2_control* enables you to control lifecycle of controllers and hardware components.
The states and transitions are the same as for the `Lifecycle Nodes`.
Check the diagram below for more details:

![Lifecycle of Hardware Interfaces](https://control.ros.org/master/_images/hardware_interface_lifecycle.png)

##### Task

1. Start the `RRbot` with `Mock System`
2. Check lifecylce state of controllers and hardware interfaces
3. Activate `joint_trajectory_controller` controller. What else you have to do to achieve that?
4. Set hardware to `inactive` state. What is now internal state of *ros2_control* instance?
5. Have you heard for `RQT Controller Manager` plugin? Try it!

##### Solution:

Branch: `6-getting-know-lifecycle`

Execute following commands to get the answers from the task:

1. `ros2 launch controlko_bringup rrbot.launch.py`
2. Execute in another terminal:
   ```
   ros2 control list_controllers
   ros2 service call /controller_manager/list_hardware_components controller_manager_msgs/srv/ListHardwareComponents {}
   ```
3. Execute the following commands (this is on of the multiple valid ways)
   ```
   ros2 control load_controller joint_trajectory_controller
   ros2 control set_controller_state joint_trajectory_controller inactive
   ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
   ```

4. Execute the following commands (this is on of the multiple valid ways)
   ```
   # stop controller
   ros2 control switch_controllers --deactivate joint_trajectory_controller
   # get component name
   ros2 service call /controller_manager/list_hardware_components controller_manager_msgs/srv/ListHardwareComponents {}
   # set component state
   ros2 service call /controller_manager/set_hardware_component_state controller_manager_msgs/srv/SetHardwareComponentState "
   name: rrbot
   target_state:
    id: 0 
    label: inactive"
   # check internals of ros2_control
   ros2 control list_controllers
   ros2 control list_hardware_interfaces
   ```


### 7. 🤖 How to write a hardware interface for a robot

Hardware interface is the lowest layer toward hardware in *ros2_control*.
A hardware interface is a driver for specific robot that exports interfaces to the framework for controllers to use them.
Overview of *ros2_control* shows this graphically:

![Overview of *ros2_control*](https://control.ros.org/master/_images/ros2_control_overview.png)

Lifecycle diagrams from the [Task 6]() explaines in detail when which method is used.

##### Task

Branch: `7-robot-hardware-interface/task`

Write a hardware interface for the *RRBot*.

1. Create or adjust a package named `controlko_hardware_interface` with hardware interface files.
2. Write a hardware interface that uses a header-only library for the communication with *RRBot*:

   - check the file `controlko_hardware_interface/include/controlko_hardware_interface/dr_denis_rrbot_comms.hpp`
   - use [Writing a new hardware interface manual](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html) to implement the everything needed.
   - extend URDF file with to use hardware interface

3. During implementation of hardware interface take care about following details:

   - Which control modes are supported?
   - What happens if an incompatible controller should be activated?

4. What are the capabilities?

**TIPP**: `RosTeamWS` tool has some scripts that can help you to solve this task faster. Resources:

  - [Commonly used robot-package structure](https://stoglrobotics.github.io/ros_team_workspace/master/guidelines/robot_package_structure.html)
  - [Creating a new package](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros_packages/create_package.html)
  - [Setup robot’s hardware package](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros2_control/setup_robot_hardware_interface.html)

##### Solution

Branch: `7-robot-hardware-interface/solution`

Execute following commands to get the answers from the task:

1. `ros2 launch controlko_bringup rrbot.launch.py use_mock_hardware:=false`

2. Test execution with `forward_position_controller` and `joint_trajectory_controller`

3. Test activation of an incompatible controller using:
   ```
   ros2 control load_controller incompatible_joint_trajectory_controller
   ros2 control set_controller_state incompatible_joint_trajectory_controller inactive
   ros2 control switch_controllers --deactivate forward_position_controller --activate incompatible_joint_trajectory_controller
   ```

8. 🛂 How to write a controller

Controllers in *ros2_control* are serving on one side as "interfaces" towards ROS-world and on the other side implement algorithms to control the hardware.
A controller, when activated, gets loaned access to exported hardware interface to read and write values directly from/to memory locations hardware interface are using.
Although somewhat limited, this concept enables deterministic and reliable data flow between controllers and hardware interfaces (drivers).

![Overview of *ros2_control*](https://control.ros.org/master/_images/ros2_control_overview.png)

##### Task

Branch: `8-write-controller/task`

Write a controller for *RRBot* robot that takes joint displacements as input and updates new joint positions for it.

1. Add files for controller names `DisplacementController` into `controlko_controllers` package.
2. During implementation of hardware interface take care about following details:

   - How is data exchanged between controller's callbacks and the `update` method?
   - How are statuses from controller published to ROS topics?

   - Controller should have a *slow mode* where displacements are reduced to the half.
   - Controller accepts a command only once.

3. Write a controller that uses `control_msgs/msg/

   - Check definition of [`JointJog` message](https://github.com/ros-controls/control_msgs/blob/galactic-devel/control_msgs/msg/JointJog.msg)
   - alternatively use CLI command: `ros2 interface show control_msgs/msg/JointJog`
   - use [Writing a new controller manuel](https://control.ros.org/master/doc/ros2_controllers/doc/writing_new_controller.html) to implement the controller


**TIPP**: `RosTeamWS` tool has some scripts that can help you to solve this task faster. Resources:

  - [Creating a new package](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros_packages/create_package.html)
  - [Setup controller package](https://stoglrobotics.github.io/ros_team_workspace/master/use-cases/ros2_control/setup_controller.html) - choose setup of "normal" controller

##### Solution

Branch: `8-write-controller/solution`

First check the code:

- *ros2_control* is now using PickNik's [generate_parameter_library](https://github.com/PickNikRobotics/generate_parameter_library) so simpler and cleaner parameters usage and definition.
  - parameters are defined in [./src/displacement_controller.yaml] file
  - example controller setup is in:
    - [./test/displacement_controller_params.yaml] - when controller is used directly with the hardware
    - [./test/displacement_controller_preceeding_params.yaml] - when controller is used at the beginning of the chain (see the next task for details!)

Execute following commands to se new controller running:

1. `ros2 launch controlko_bringup rrbot.launch.py`

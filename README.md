# arm_api2

API for the robotic manipulators realying on: 
* [Moveit2](https://moveit.picknik.ai/main/index.html)
* [ROS 2](https://docs.ros.org/en/humble/index.html)

Docker for building required environment can be found [here](https://github.com/larics/docker_files/tree/master/ros2-humble/moveit2). 

For building ROS 2 packages and moveit, it is neccessary to use [colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html). 

#### Build just one package: 

```
colcon build --packages-select arm_api2
```

Build with the compile commands: 
```
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Building with `--symlink-install` causes it to fail often, you can run: 
```
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --continue-on-error
```

Full verbose build command: 
```
colcon build --symlink-install --packages-select moveit2_tutorials --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_VERBOSE_MAKEFILE=ON
```

### Aliases

Copy to `~/.bashrc` and source it. 

```
alias cbp="colcon build --symlink-install --packages-select" 
alias panda_sim="ros2 launch panda gz.launch.py"
alias kinova_sim="ros2 launch kortex_bringup kortex_sim_control.launch.py dof:=7 use_sim_time:=true launch_rviz:=false" 

```

Build `arm_api2` package as: 

```
cbp arm_api2
``` 

Start franka with: 

```
franka_sim
```

Start kinova with: 

```
kinova_sim
```
### Aim of the repository

With `arm_api` as precursor, which was intended to provide simple ROS interfacing with 
robot manipulators with the help of moveit. 

This repository `arm_api2` is intended to provide interfacing support for robot manipulators for ROS 2 and MoveIt2!. 

### Change state 

In order to change state call following command: 
```
ros2 service call /change_state arm_api2_msgs/srv/ChangeState "{state: JOINT_TRAJ_CTL}
```

### Goals 

Create API simple to **run** and **maintain** that supports working with different 
robot manipulators out of the box. 

Robot manipulators of interest are: 
* Franka Emika
* UR 
* Kinova 
* Kuka 

Wanted arm functionalities: 
1. `go_to_pose`
2. `grasp` 

### Launch

Launch ign simulation with following command: 
```
ros2 launch panda ign.launch.py
```



<details>
<summary><h3> Dev help </summary>

You can add `RvizVisualToolsGui` with `Add New Panel` in the RVIZ2. 
</details>

<details>
<summary><h2> Arm interfaces </h2></summary>

* [franka_ros2](https://support.franka.de/docs/franka_ros2.html)
* [kinova_ros2](https://github.com/Kinovarobotics/ros2_kortex)
* [UR_ros2](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
</details>


<details>
<summary><h2>Status</h2></summary>

### DONE: 
- [x] Create pkg skeleton 
- [x] Build moveit2 and ros 2 Docker 
- [x] Decouple header and source
- [x] Create first publisher and subscriber 
- [x] Init MoveGroup
- [x] Add ign running
- [x] Add ctl 
- [x] Add change state
- [x] Init planning scene
- [x] Enable autocomplete with compile_commands
- [x] Implement first arm_api2 for the franka for ros 2
- [x] Init planning scene
- [x] Added kinova_sim
- [x] Added franka_sim
- [x] Added init cartesian control 
- [x] Added init servo control 
- [x] Added init joystick but add teleop one
- [x] Init planning scene
- [x] Autocomplete with compile commands
- [x] First arm api for for franka_ros

### TODO: 
- [ ] Define SW patterns that makes sense to use
- [ ] Init planning scene 
- [ ] Add joystick 
</details>


<details> 
<summary><h3> Useful learning links</h3></summary>

- [Declare variables as const](https://www.cppstories.com/2016/12/please-declare-your-variables-as-const/)
- [Complicated variable initialization](https://www.cppstories.com/2016/11/iife-for-complex-initialization/)
- [C++ good practices](https://ctu-mrs.github.io/docs/introduction/c_to_cpp.html)
- [MoveIt2! C++ iface](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html)
- [How to setup VSCode](https://picknik.ai/vscode/docker/ros2/2024/01/23/ROS2-and-VSCode.html)
- [First Cpp node for ROS 2](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/first_node_cpp.html) 
- [Composition of ROS nodes](https://answers.ros.org/question/316870/ros2-composition-and-node-names-with-launch-files/)
- [planning_scene](https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/planning_scene/src/planning_scene_tutorial.cpp)
- [custom moveit ns](https://github.com/moveit/moveit2/issues/2415)
- [publish robot_description](https://github.com/moveit/moveit2_tutorials/issues/525)
- [joint state clock not in sync](https://answers.ros.org/question/417209/how-to-extract-position-of-the-gripper-in-ros2moveit2/)
- [issue for initializing MGI](https://github.com/moveit/moveit2/issues/496)

</details>


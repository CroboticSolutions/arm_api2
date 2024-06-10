# arm_api2

API for robotic manipulators based on: 
* [Moveit2](https://moveit.picknik.ai/main/index.html)
* [ROS 2](https://docs.ros.org/en/humble/index.html)

Docker for building required environment can be found [here](https://github.com/CroboticSolutions/docker_files/tree/master/ros2/humble/kinova). 

For building ROS 2 packages and moveit, it is neccessary to use [colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html). 

### Depends on: 
- [arm_api2_msgs](https://github.com/CroboticSolutions/arm_api2_msgs)

Aditional dependencies are: 
- [kinova](https://github.com/CroboticSolutions/ros2_kortex)
- [panda](https://github.com/AndrejOrsula/panda_ign_moveit2)

### Build:

Build in ROS 2 workspace. 
Build just one package with: 
```
colcon build --packages-select arm_api2
```

Build with the compile commands (enable autocomplete): 
```
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Building with `--symlink-install` causes it to fail often because of already built ROS 2 packages, you can run: 
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
3. `release` 
4. `push`
5. `<something_else?>`

### Launch

Launch ign simulation with following command: 
```
ros2 launch panda ign.launch.py
```

<details>
<summary><h3>How to?</summary>

You can run kinova in simulation by executing following commands: 
```
ros2 launch kortex_bringup kortex_sim_control.launch.py dof:=7 use_sim_time:=true launch_rviz:=false
```
or 
```
kinova_sim
```
if alias has been added. 

After that run `move_group` node as follows: 
```
ros2 launch kinova_gen3_7dof_robotiq_2f_85_moveit_config sim.launch.py
```

After that run `arm_api2` `moveit2_iface` node as follows: 
```
ros2 launch arm_api2 moveit2_iface.launch.py 
```

### ROS 2 robot interface: 

**Change robot state**: 
- srv: `/arm_api2_msgs/srv/ChangeState.srv`
- values `JOINT_TRAJ_CTL || CART_TRAJ_CTL || SERVO_CTL`

**Command robot pose**: 
- msg: `geometry_msgs/msg/PoseStamped.msg`

**Command cartesian path**:   
- msg: `arm_api2_msgs/msg/CartesianWaypoints.msg`

**Get current end effector pose**: 
- msg: `geometry_msgs/msg/PoseStamped.msg`

#### Available robot states

Currently implemented and available robot states are: 
- `JOINT_TRAJ_CTL`, which is used for joint control 
- `CART_TRAJ_CTL`, which is used for cartesian control 
- `SERVO_CTL`, which is used for the end effector servoing


#### Kinova

How to setup kinova [here](https://git.initrobots.ca/amercader/kinova-kortex-installation). 


</details>

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

### TODO: 
- [ ] Test with real robot manipulators
- [ ] Create standardized joystick class [full]
- [ ] Discuss potential SW patterns that can be used
- [ ] Update and fix documentation
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


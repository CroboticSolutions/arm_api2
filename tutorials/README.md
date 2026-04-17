# Pick and place with **arm_api2**

This tutorial walks through a minimal **pick-and-place** sequence on a simulated Universal Robots arm using **arm_api2**. The same pattern applies to other supported manipulators: a small set of **topics** and **services** replaces bespoke MoveIt client code, so application logic stays short and maintainable.

**Why arm_api2 for pick and place**

| Without a unified API | With arm_api2 |
|----------------------|---------------|
| Wire MoveIt actions, planners, and frame logic per application | Send **pose commands** on `/arm/cmd/pose` and switch control mode with **one service** |
| Reimplement joint vs Cartesian flows | Toggle `JOINT_TRAJ_CTL` vs `CART_TRAJ_CTL` when you need axis-aligned descent |
| Ad-hoc gripper integration | Call **`/arm/open_gripper`** and **`/arm/close_gripper`** (service types as in your config) |

You focus on **waypoints** (home, approach, pick, place, retract)—not on plumbing.

---

## Prerequisites

- ROS 2 workspace with `arm_api2`, `ur_simulation_gz`, and `ur_moveit_config` built and sourced.
- This example uses namespace **`ur1`** and planning frame **`world`**, matching `robots_profile:=lab_gripper_one` (UR + Robotiq + `lab_table_one` world with three cubes).

---

## 1. Start simulation and MoveIt

Terminal 1 — Gazebo + MoveIt + configured world (red / blue / yellow cubes):

```bash
ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py robots_profile:=lab_gripper_one
```

Wait until MoveIt and controllers are up before continuing.

---

## 2. Start arm_api2 (simple interface)

Terminal 2:

```bash
ros2 launch arm_api2 moveit2_simple_iface.launch.py \
  robot_name:=ur robot_ns:=ur1 use_sim_time:=true
```

All commands below use the **`/ur1`** prefix. Adjust if your namespace differs.

---

## 3. Pick-and-place sequence (conceptual steps)

1. **Home** — Move the arm to a safe starting pose (joint-space motion).
2. **Approach** — Move above the first cube (same orientation, higher *z*).
3. **Pick** — Switch to Cartesian control; descend along *z*; close the gripper.
4. **Retract** — Lift clear of the table.
5. **Move to place** — Joint motion to approach above the drop pose.
6. **Place** — Cartesian descent; open the gripper; retract.

The numbered commands below implement that flow. Numeric poses are **example** values for the bundled world; tune for your calibration.

---

### 3.1 Move to home (`JOINT_TRAJ_CTL`)

Put the arm in joint trajectory mode, then send a single pose target (joint-space planning behind the scenes):

```bash
ros2 service call /ur1/arm/change_state arm_api2_msgs/srv/ChangeState "{state: 'JOINT_TRAJ_CTL'}"
```

```bash
ros2 topic pub --once /ur1/arm/cmd/pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.08571, y: 0.2951, z: 1.664}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

---

### 3.2 Approach above the first cube

```bash
ros2 topic pub --once /ur1/arm/cmd/pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.3988, y: 0.5512, z: 1.54}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

---

### 3.3 Pick — Cartesian descent and grasp

Switch to Cartesian trajectory control for straight-line motion (here, along *z*):

```bash
ros2 service call /ur1/arm/change_state arm_api2_msgs/srv/ChangeState "{state: 'CART_TRAJ_CTL'}"
```

```bash
ros2 topic pub --once /ur1/arm/cmd/pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.3988, y: 0.5512, z: 1.4767}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

Close the gripper:

```bash
ros2 service call /ur1/arm/close_gripper std_srvs/srv/Trigger {}
```

---

### 3.4 Retract (lift)

```bash
ros2 topic pub --once /ur1/arm/cmd/pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.3988, y: 0.5512, z: 1.54}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

---

### 3.5 Move to place — approach above drop location

Return to joint mode for the horizontal move to the place station:

```bash
ros2 service call /ur1/arm/change_state arm_api2_msgs/srv/ChangeState "{state: 'JOINT_TRAJ_CTL'}"
```

```bash
ros2 topic pub --once /ur1/arm/cmd/pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.04671, y: 0.5512, z: 1.54}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

---

### 3.6 Place — descend and release

```bash
ros2 service call /ur1/arm/change_state arm_api2_msgs/srv/ChangeState "{state: 'CART_TRAJ_CTL'}"
```

```bash
ros2 topic pub --once /ur1/arm/cmd/pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.04671, y: 0.5512, z: 1.4767}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

```bash
ros2 service call /ur1/arm/open_gripper std_srvs/srv/Trigger {}
```

Retract after place:

```bash
ros2 topic pub --once /ur1/arm/cmd/pose geometry_msgs/msg/PoseStamped \
"{header: {frame_id: 'world'}, pose: {position: {x: 0.04671, y: 0.5512, z: 1.54}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"
```

---

## 4. Next steps

- Automate the same sequence in **Python** or **C++** by publishing the same message types; no change to the arm_api2 contract.
- For action-based flows (goals with feedback), use `moveit2_iface.launch.py` instead of the simple interface.
- See the main [arm_api2 README](../README.md) for `ChangeState`, velocity scaling, and end-effector frame options.

---

*World layout: `lab_gripper_one` / `lab_table_one.sdf`. Re-verify poses if you change the world or robot profile.*

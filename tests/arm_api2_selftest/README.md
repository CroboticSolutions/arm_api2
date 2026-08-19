# arm_api2 selftest

Functional smoke test for a running `moveit2_iface` node. Exercises the
services, topics and actions documented in the top-level README and prints
a PASS/FAIL/SKIP report with a non-zero exit code on failure.

All motion checks are **relative nudges** computed from the robot's current
pose/joint state when the test starts - there are no per-robot absolute
poses to keep in sync with a particular world/scene.

## What it checks

With `--move`, the first physical-motion step (after baseline/control-state
health checks) moves the arm to the configured SRDF named state (`warmup_group_state`, default
`"test_configuration"`) before anything else runs. This isn't just a
fallback: the robot's default spawn pose can itself be a genuine kinematic
singularity, and the simulation is **not** reset between separate selftest
invocations - a fresh run starts wherever the *previous* run's real-motion
tests happened to leave the arm, an arbitrary pose either way. Starting every
`--move` run from the same known-good pose makes results reproducible instead
of depending on simulation history. (The reactive probing described under
"Cartesian direction probing" below still runs later in the suite too, as a
safety net - a chain of small real moves can drift back into a locally
awkward configuration even after starting clean here.)

Always run (no physical motion by default):

- `arm/state/current_pose`, `joint_states`, `arm/state/ctl_state` topics are alive
- `arm/change_state` invalid-input rejection and `arm/state/ctl_state` consistency
- `arm/set_vel_acc`, including rejection of negative and non-finite input
- `arm/set_eelink` (only if `eelink_default` is set in the config - see below)
- `arm/check_reachability`
- `arm/check_cartesian_path` - probes 6 axis directions from the current pose (see
  "Cartesian direction probing" below); with `--move`, may trigger a small
  warm-up move to the SRDF `test_configuration` state if none of the 6 pass
  (e.g. the arm spawned at/near a singularity) before failing for real
- `arm/set_planner` x `arm/move_to_joint`, with `arm/set_planonly` enabled - verifies
  each plan succeeds **and** that every arm joint actually stayed put
  afterwards (planonly's whole contract), not just that the action reported success

Only with `--move` (commands real motion - use in sim first):

- `arm/open_gripper`, `arm/close_gripper` and the standard
  `arm/gripper_control` action (skip with `skip.gripper: true` for gripper-less robots)
- `JOINT_TRAJ_CTL` + `arm/move_to_joint` - verifies the commanded joint actually
  travelled and all arm joints converged. The test reads URDF limits, chooses a
  direction with margin, and then returns exactly to the captured baseline
- `CART_TRAJ_CTL` + `arm/move_to_pose` - waits for `arm/state/current_pose` to
  actually converge on the target pose
- `arm/move_to_pose_path` - waypoints are a small **bend** (out along the probed
  direction, then onto a second, non-parallel axis) rather than a straight
  out-and-back. A pure there-and-back path reverses ~180 degrees at its
  turnaround waypoint, which MoveIt's time-optimal trajectory generation
  cannot parameterize ("The path requires a 180 deg. turn, which is not
  supported by the current implementation") - a known, documented MoveIt
  limitation (see `moveit_core/trajectory_processing/time_optimal_trajectory_generation.cpp`),
  not an arm_api2 defect. The bend axis itself is validated via
  `check_cartesian_path` from the first waypoint before use (same as the
  primary direction) - an untested bend axis can be just as infeasible
  locally as most of the 6 candidate axes are from the starting pose, in
  which case `computeCartesianPath` silently truncates the path at the first
  waypoint every single time, no matter how many times it's retried
- `arm/cmd/pose` (topic-based "simple interface", `pose_cmd_cb`) - a separate code
  path from the `move_to_pose` action above. Its handler (`planTopicPoseFast`)
  only tries `pilz_LIN` then `ompl_EST` with short time budgets, ignoring
  whatever `arm/set_planner` configured, and a command is consumed once -
  silently dropped if it arrives while a previous move is still finishing
  ("Trajectory still executing; ignoring...") or if planning fails outright.
  `_publish_and_wait_for_pose` (see below) republishes as soon as it notices
  the arm isn't making progress, instead of waiting out a long fixed timeout
  in silence. Both its `CART_TRAJ_CTL` and `JOINT_TRAJ_CTL` implementations are tested
- `arm/cmd/traj` (topic-based Cartesian waypoints, `cart_poses_cb`) - separate
  code path from the `move_to_pose_path` action, same validated bend-not-reversal
  waypoints and same progress-aware republish behavior
- `SERVO_CTL` + a short twist burst on `moveit2_iface/servo_twist_cmd` - measures
  real Cartesian displacement during the burst, per candidate direction (see
  below). Also subscribes to `moveit2_iface/status`: if every direction
  measures ~0 displacement AND moveit_servo's own status reported
  INVALID/HALT_FOR_SINGULARITY/HALT_FOR_COLLISION, that's servo's safety guard
  correctly refusing to move from a bad pose, not a defect - the same SRDF
  warm-up used elsewhere kicks in before this is treated as a real failure

Note: `mode:=advanced` (the default) keeps the topic interface live alongside
the actions, so `arm/cmd/pose` / `arm/cmd/traj` are tested against the same
running node as everything else - no separate `mode:=simple` launch needed.

Every real-motion check validates against the robot's own reported state
(`joint_states` / `arm/state/current_pose`) after the fact - an action or service
reporting `success` is never treated as proof by itself that the arm moved.
Convergence requires multiple **new** state messages; repeatedly reading one
cached sample cannot produce a PASS. If the initial SRDF warm-up fails, all
remaining physical-motion checks are blocked rather than continuing from an
unknown pose.

Action timeouts request cancellation and wait for acknowledgement before the
suite continues. On shutdown, the selftest also cancels an active action and
best-effort restores the control state that was active when it started.

### Cartesian direction probing

`check_cartesian_path`, `move_to_pose` and `move_to_pose_path` share one helper:
try a small nudge along each of the 6 axis directions via `check_cartesian_path`
and use the first one that validates. The servo test does its **own**, separate
6-direction search measuring actual Cartesian displacement per direction rather
than reusing this one - a direction that `check_cartesian_path` accepts only
proves an IK solution exists there, not that it has good manipulability for
*velocity* control, and MoveIt Servo silently scales down its output near a
low-manipulability direction or joint limit without raising a halt status. A
direction picked this way could otherwise pass `check_cartesian_path` but make
the servo test measure near-zero movement despite servoing working correctly.

A freshly-spawned
arm can be sitting at/near a singularity or a workspace boundary (e.g. a fully
extended elbow) where every direction from the exact current pose fails - this
is a real property of that pose, not a bug. If all 6 fail and `--move` is set,
the helper reads the named SRDF `<group_state>` given by `warmup_group_state`
(default `"test_configuration"`, read live from `move_group`'s
`robot_description_semantic` - no joint values hard-coded here) and moves the
arm there via `arm/move_to_joint` before probing again. Without `--move`, or if
the named state doesn't exist for a given robot, a persistent 6/6 failure is
reported as a real, unmasked FAIL.

### Progress-aware republish for single-shot topic commands

`arm/cmd/pose` and `arm/cmd/traj` are fire-and-forget at the command topic, but
the node reports their outcome on `arm/state/plan_status`.
`_publish_and_wait_for_pose` correlates fresh plan-status messages with the
requested pose and combines that with distance-to-target:

- If distance-to-target is shrinking, the move is genuinely in progress -
  keep waiting, even through a multi-waypoint detour (e.g. `arm/cmd/traj`'s
  bend passes through an intermediate waypoint before the final one).
- If distance-to-target hasn't shrunk over a few seconds, the last publish
  was likely dropped (or, for `arm/cmd/pose`, `planTopicPoseFast`'s
  sampling-based `ompl_EST` attempt failed) - republish immediately instead
  of waiting out a long fixed timeout in silence. Retries are bounded by
  `timeouts.topic_max_retries`; non-retryable planning/execution errors fail immediately.

A cruder "republish if no raw movement" check was tried first and made things
worse: it would republish an already-succeeding multi-waypoint move partway
through (any waypoint transition briefly looks like "not at the final target
yet"), restarting a move that was about to finish - forever, in the worst
case. Tracking net progress toward the target survives that detour.

Not automated (need physical input, reported as SKIP with instructions):

- `joy_ctl` / `keyboard_ctl` - launch them and manually confirm the arm follows
  your joystick/keyboard input in `SERVO_CTL` mode.
- `servo_watchdog.py` - stop sending twist/jog commands while it is running and
  confirm (via `ros2 topic echo <ns>/moveit2_iface_node/delta_twist_cmds` or the
  node's log) that it publishes a zero-velocity command after a few seconds of
  inactivity.

## How to run

1. Build the workspace so `arm_api2_selftest.py` and its config are installed:

   ```bash
   colcon build --packages-select arm_api2
   source install/setup.bash
   ```

2. Start your robot simulation + MoveIt, and `moveit2_iface`. For UR, use the
   same launch as the pick-and-place tutorial (a single combined launch that
   also brings up the gripper, world and RViz in one go) rather than the
   separate `ur_sim_control.launch.py` + `ur_moveit.launch.py` pair - and use
   **`mode:=advanced`** (or leave `mode` unset, it's the default), not the
   tutorial's `mode:=simple`, since the selftest needs the action servers that
   `simple` mode disables:

   ```bash
   ros2 launch ur_simulation_gz multi_ur_sim_moveit.launch.py robots_profile:=lab_gripper_one
   ros2 launch arm_api2 moveit2_iface.launch.py \
     robot_name:=ur robot_ns:=ur1 use_sim_time:=true mode:=advanced
   ```

   `use_sim_time:=true` matters: without it, `moveit2_iface`'s clock disagrees
   with Gazebo's simulated clock and MoveGroupInterface fails to fetch the
   robot state entirely.

3. In a third terminal, run the safe (no-motion) checks first:

   ```bash
   ros2 run arm_api2 arm_api2_selftest.py --robot-ns ur1
   ```

4. Once that's green, allow real motion (make sure the robot has clear space
   around it - the arm and gripper will move; the arm uses ~2cm Cartesian
   nudges and a bit less than 3 degrees on one joint,
   plus possibly a one-off move to the `test_configuration` SRDF pose if it
   spawned at/near a singularity):

   ```bash
   ros2 run arm_api2 arm_api2_selftest.py --robot-ns ur1 --move
   ```

## Customizing per robot

Copy `selftest_defaults.yaml`, adjust it, and pass it as the first argument -
or start from one of the ready-made overrides in `examples/` (e.g. `examples/ur.yaml`
for the UR + Robotiq sim config, which just sets `eelink_default: "tool0"` to
match `config/ur/ur_sim.yaml`'s `robot.ee_link_name`):

```bash
ros2 run arm_api2 arm_api2_selftest.py my_selftest.yaml --move
```

Notably:

- `skip.gripper: true` for robots without a gripper configured.
- `skip.servo: true` if `moveit_servo` isn't enabled for that robot.
- `eelink_default: "<link name>"` (matching `robot.ee_link_name` in that
  robot's `config/<robot>/<robot>_<profile>.yaml`) to enable the
  `arm/set_eelink` check - it is set back to this same value, so there is no
  side effect on the rest of the run.
- `warmup_group_state: "<SRDF group_state name>"` used by the Cartesian
  direction probe (see above) - `"test_configuration"` by default, `""` to
  disable the warm-up fallback entirely.
- `planners:` to match what's actually installed (e.g. add `"cumotion"` if the
  cuMotion planner plugin is present in your image).
- `arm_controller_name: "<controller>"` to avoid heuristic controller discovery
  when several unrelated ros2_control controllers are active.
- `timeouts.state_freshness_sec` controls how old baseline state may be;
  `timeouts.topic_max_retries` bounds fire-and-forget topic retries.

The service API currently has no getter for the previous velocity/acceleration
scaling or planner selection. The selftest therefore leaves the configured
`vel_acc_max_vel`, `vel_acc_max_acc`, and final tested planner active; configure
those values to settings acceptable for the robot under test.

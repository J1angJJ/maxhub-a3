# carm_a3_description

ROS Noetic description package for CArm / MAXHUB A3.

This package was migrated from the downloaded `carm_a3.zip` model package. The original package was ROS 2 / ament style; this version is converted to ROS 1 / catkin for the current Noetic workspace.

## Contents

- `urdf/carm_a3.urdf`: robot model.
- `meshes/*.STL`: visual and collision meshes.
- `bind/joints_bind.json`: joint binding reference from the source model.
- `launch/robot_state_publisher.launch`: publish TF from `/joint_states`.
- `launch/display.launch`: standalone RViz display using `joint_state_publisher`.

## TF

The URDF publishes the full joint chain from `base_link` through the arm links. A fixed `flange` link has been added under `link6` so the hand-eye transform can attach to:

```text
flange -> carm_a3_camera_optical_frame
```

In normal bringup, `carm_a3_driver` publishes `/joint_states`, while `robot_state_publisher` publishes the full robot TF chain.

Current `/joint_states` contains `joint1` through `joint6`. The arm chain up to `flange` is therefore available. Gripper finger joints `joint7` and `joint8` require future gripper state publishing before their moving TF is meaningful.

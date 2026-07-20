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

Current `/joint_states` contains `joint1` through `joint6`, and `carm_a3_motion` can append `joint7` and `joint8` from the SDK gripper gap. The arm chain up to `flange` is therefore available, and the finger links become meaningful when the unified motion node is running with `publish_gripper_joints:=true`.

The model also defines a draft `gripper_tcp` frame under `gripper_base`. The `flange -> gripper_base` transform is derived from the full-machine STEP assembly, while `gripper_base -> gripper_tcp` is still an estimated two-finger center near the front of the gripper. Validate it against the real fingertip/contact point before using it for table-contact grasping:

```text
flange -> gripper_base -> gripper_tcp
```

See `docs/vendor/a3_full_model_step_probe.md` for the STEP-derived candidate values.

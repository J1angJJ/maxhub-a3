# carm_a3_tasks

Task-level helpers for grasp preparation.

The first helper plans a pre-grasp overview pose for the 60x60 cm table region in front of the arm. It does not move by default.

Assumption:

- `base_link` origin is at the midpoint of the near edge of the square table region.
- Workspace is `x: 0.0..0.60 m`, `y: -0.30..0.30 m`.
- The overview target starts above the region center at `[0.30, 0.0, 0.36]`.
- Orientation defaults to the current flange orientation for the first pass.

## Plan Only

```bash
rosrun carm_a3_tasks grasp_init.py plan
```

## Execute

Only after the plan output looks reasonable and the robot is clear:

```bash
rosrun carm_a3_tasks grasp_init.py execute --max-joint-delta 0.35
```

Execution splits the final joint target into small waypoints using `segment_delta_rad`, so each step still goes through the motion service's joint-delta and verification checks.

The script calls:

- `/carm_a3/motion/get_cartesian_snapshot`
- `/carm_a3/motion/get_joint_snapshot`
- `/carm_a3/motion/solve_ik`
- `/carm_a3/motion/move_joint` only with `execute`

After execution, inspect the camera view:

```bash
roslaunch carm_a3_bringup readonly_vision_handeye.launch launch_color_blocks:=true
rosrun image_view image_view image:=/carm_a3/perception/color_blocks/debug_image
```

If the table is not centered in view, tune `config/grasp_init.yaml` before using this as the grasp workflow's fixed start pose.

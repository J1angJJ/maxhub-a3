# carm_a3_tasks

Task-level helpers for grasp preparation.

The first helper plans a pre-grasp overview pose for the 60x60 cm table region in front of the arm. It does not move by default.

By default it solves the overview pose from the current camera intrinsics and hand-eye calibration, so the wrist camera should look down and cover the configured table region before color-block grasping starts.

Assumption:

- `base_link` origin is at the midpoint of the near edge of the square table region.
- Workspace is `x: 0.0..0.60 m`, `y: -0.30..0.30 m`.
- Camera intrinsics are loaded from `carm_a3_vision/config/camera_info.yaml`.
- Hand-eye is loaded from `carm_a3_calibration/config/handeye_flange_camera.yaml`.
- FOV mode adds `coverage_margin_m` around the table, computes the needed camera height, then converts the camera pose to a flange target.
- If the ideal straight-down FOV pose has no IK solution, `plan` can search nearby camera centers and tilted look-at poses, then choose a reachable candidate.

## Start Overview Camera Stack

This launch starts the robot model TF, camera, hand-eye TF, color block segmentation, and an `image_view` window. It does not start the motion SDK node by default, so it will not collide with the real motion gate:

```bash
roslaunch carm_a3_tasks pregrasp_overview.launch
```

Keep this running while planning or executing the pre-grasp pose.

Start exactly one motion SDK node in another terminal when you need planning services or execution:

```bash
roslaunch carm_a3_motion safe_motion.launch \
  allow_motion:=true \
  dry_run:=false \
  auto_ready_on_connect:=true \
  register_callbacks_on_connect:=true \
  pre_ready_delay_s:=1.0
```

To only print the FOV-derived target without calling IK:

```bash
rosrun carm_a3_tasks grasp_init.py fov
```

## Plan Only

```bash
rosrun carm_a3_tasks grasp_init.py plan
```

## Execute

Only after the plan output looks reasonable and the robot is clear:

```bash
rosrun carm_a3_tasks grasp_init.py execute
```

Execution checks the total joint distance with `max_total_joint_delta_rad`, then splits the final joint target into small waypoints using `segment_delta_rad`, so each step still goes through the motion service's joint-delta and verification checks.

The script calls:

- `/carm_a3/motion/get_cartesian_snapshot`
- `/carm_a3/motion/get_joint_snapshot`
- `/carm_a3/motion/solve_ik`
- `/carm_a3/motion/move_joint` only with `execute`

The older one-shot launch also starts the camera stack and image window by default, then runs the selected command. It also leaves motion disabled unless `launch_motion:=true` is passed explicitly:

```bash
roslaunch carm_a3_tasks grasp_init.launch command:=plan
```

If the table is not centered in view, tune `config/grasp_init.yaml` before using this as the grasp workflow's fixed start pose. If the log says the required camera height exceeds `max_camera_height_m`, the requested FOV coverage is geometrically larger than the configured height allows; reduce margin, accept partial coverage, or choose a different viewing orientation.

Useful search parameters:

- `overview/search_reachable`: enable IK-backed fallback search.
- `overview/search_height_step_m`: vertical scan step.
- `overview/search_x_offsets_m`: camera center offsets along table X.
- `overview/search_y_offsets_m`: camera center offsets along table Y.
- `overview/max_total_joint_delta_rad`: maximum full initialization joint distance.
- `overview/segment_delta_rad`: maximum interpolation step size before each `move_joint` service call.

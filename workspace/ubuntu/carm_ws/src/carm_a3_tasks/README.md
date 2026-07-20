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
  allow_gripper:=true \
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

`execute` closes the gripper to `gripper/overview_pos_m` before moving to reduce camera occlusion. The default is now `0.005 m`; disable with `gripper/close_before_overview: false` if needed.

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
- `gripper/close_before_overview`: close the gripper before moving to the overview pose.
- `gripper/overview_pos_m`: gripper gap used for overview.
- `gripper/overview_tau_n`: conservative gripping force used for overview.

## Color Block Grasp Draft

`block_grasp.py` reads the red/green block detection JSON, projects the selected pixel center through the calibrated wrist camera into the table plane, then plans approach, grasp, and lift poses. It does not execute by default.

Start the camera/perception window and keep one real motion gate running in another terminal, then plan:

```bash
rosrun carm_a3_tasks block_grasp.py plan
rosrun carm_a3_tasks block_grasp.py plan --color red
rosrun carm_a3_tasks block_grasp.py plan --color green
```

The launch form loads `config/block_grasp.yaml`:

```bash
roslaunch carm_a3_tasks block_grasp.launch command:=plan color:=red
roslaunch carm_a3_tasks block_grasp.launch command:=plan color:=green extra_args:="--allow-descend --use-gripper"
```

After checking the printed base-frame block point, poses, IK results, and clearance, the first execution only moves to the approach pose:

```bash
rosrun carm_a3_tasks block_grasp.py execute --color red
```

Descent to the grasp pose is disabled by default because the current model does not yet include a validated gripper TCP/contact height. In the default mode, `plan` and `execute` only validate and solve the approach pose. Only enable descent after the approach pose is visibly safe:

```bash
rosrun carm_a3_tasks block_grasp.py execute --color red --allow-descend
```

Gripper open/close is intentionally separate and also requires descent to be enabled:

```bash
rosrun carm_a3_tasks block_grasp.py execute --color red --allow-descend --use-gripper
```

For real grasp tests, prefer the launch form so `config/block_grasp.yaml` is loaded:

```bash
roslaunch carm_a3_tasks block_grasp.launch command:=execute color:=green extra_args:="--allow-descend --use-gripper"
```

The first version keeps the current flange orientation and uses conservative fixed heights from `config/block_grasp.yaml`. Treat it as a grasp-chain smoke test before tuning the true TCP, grasp height, and gripper width. `grasp/min_flange_z_m` is a hard planning guard against near-table flange targets.

By default, the approach pose opens the gripper before moving and estimates both long/short block axes from the detected rectangle corners. Because the block is `5 x 5 x 10 cm` and the gripper opens to about `8 cm`, `grasp/auto_select_grasp_side` selects the gripper span direction over the 5 cm side instead of the 10 cm long side. If the gripper is visually 90 degrees off, change `grasp/align_yaw_offset_deg` or switch `grasp/align_tool_axis` between `x` and `y`.

`grasp/use_tcp_target` makes the planner treat the detected block point as a TCP target and converts it back to the required flange pose with `grasp/flange_to_tcp_xyz_m`. Tune `grasp/tcp_grasp_z_m` first when the gripper is too high or too low.

The gripper span axis is bidirectional by default, so a detected rectangle edge can align with either `+axis` or `-axis` without forcing a 180 degree wrist flip.

Approach-only execution keeps the current overview orientation by default. This makes the first XY/TCP alignment check independent from wrist alignment and avoids unnecessary IK branch changes. Add `--allow-descend` to enable gripper-axis alignment for the real grasp sequence.

After approach-only motion, `grasp/visual_recenter_after_approach` can re-detect the block, compare its projected base-frame center with the current `gripper_tcp`, and execute bounded XY corrections. This estimates the offset automatically instead of hard-coding a manual target shift.

Full grasp execution uses the same idea by default. With `--allow-descend --use-gripper`, the task first moves to an approach pose that keeps the camera view stable, runs visual recentering, then re-detects the block and replans the aligned approach, descent, and lift from the real corrected state. The aligned stage uses `grasp/align_transit_waypoints` quaternion interpolation waypoints to reduce sudden IK branch jumps while rotating the gripper toward the selected short side.

For real motion, trajectory fallback is disabled by default. If a continuous trajectory partly moves but fails verification, the task stops instead of retrying from stale planned waypoints. `grasp/max_segment_joint_delta_rad` also rejects large IK branch jumps during planning.

The move from the overview pose to approach also inserts `grasp/view_transit_waypoints` intermediate IK targets. These waypoints keep the current overview orientation while translating toward the block, so the wrist camera tends to keep the block in view during the approach instead of swinging away through a pure joint-space interpolation.

Both overview initialization and block approach prefer `/carm_a3/motion/move_joint_trajectory` when available. The motion node still checks each adjacent waypoint against its joint-delta limit, but sends the whole waypoint list to the SDK trajectory API for smoother execution. Set `prefer_joint_trajectory: false` to fall back to segmented `move_joint`.

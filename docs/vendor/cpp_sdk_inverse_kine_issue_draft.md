# C++ SDK `inverse_kine` fails on FK/current pose round-trip

## Environment

- OS: Ubuntu 20.04
- ROS: Noetic
- Controller/model reported by SDK/WebSocket: `A3_DM_C`
- Controller: `192.168.31.60:8090`
- SDK location in test workspace: `workspace/ubuntu/carm_ws/vendor/arm_control_sdk`

## Summary

`forward_kine()` works, and `get_cart_pose()` / `get_plan_cart_pose()` return the same current Cartesian pose. However, `inverse_kine()` returns `-1` even when the target pose is generated from the current joint state by `forward_kine()`, using the current joint state as the reference seed.

This means a basic FK -> IK round-trip fails.

## Observed Current State

Current joints:

```text
[0.0197496, 0, 0, -0.000953674, -0.000190735, 0.000190735]
```

Current Cartesian pose from `get_cart_pose()`:

```text
[-2.9254701416903117e-07, 1.3870500303880817e-08, 0.23601900041103363,
 0.7070009708404541, 0.007251520175486803, 0.707144021987915,
 -0.006713429931551218]
```

`forward_kine(tool_index=0, current_joints)` succeeds:

```text
forward_kine ret=1
pose=[-2.92547e-07, 1.38705e-08, 0.236019,
      0.707001, 0.00725151, 0.707144, -0.00671341]
```

The FK pose matches `get_cart_pose()`.

## IK Probe Result

The following candidates all return `inverse_kine ret=-1` with `solution=[]` for `tool_index=0,1,2,3`:

- `get_cart_pose()`
- `get_plan_cart_pose()`
- `forward_kine(current_joints)`
- quaternion sign variants

## Expected Behavior

At least `inverse_kine(tool_index, forward_kine(tool_index, current_joints), current_joints)` should return a valid solution close to `current_joints`, or the SDK should document additional requirements for tool index, pose order, units, or valid seed/reference configuration.

## Local Follow-up

The local diagnostic CLI has been extended to try more pose convention variants:

```bash
rosrun carm_a3_motion motion_cli.py ik-probe
rosrun carm_a3_motion motion_cli.py ik-probe --include-mm
```

`--include-mm` is only a read-only IK diagnostic. It checks whether the inverse kinematics path expects millimeter-position inputs despite FK/cartesian outputs appearing to be in meters.

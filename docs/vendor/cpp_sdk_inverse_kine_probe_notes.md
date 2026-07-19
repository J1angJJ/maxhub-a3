# C++ SDK `inverse_kine` diagnostic notes

This file was first created as a possible vendor issue draft, but the latest probe shows that the C++ SDK inverse kinematics path can work. Do not submit this as a bug report in its current form.

## Environment

- OS: Ubuntu 20.04
- ROS: Noetic
- Controller/model reported by SDK/WebSocket: `A3_DM_C`
- Controller: `192.168.31.60:8090`
- SDK location in test workspace: `workspace/ubuntu/carm_ws/vendor/arm_control_sdk`

## Current Conclusion

`forward_kine()` works, and `inverse_kine()` can solve the current Cartesian pose when called with:

- position units: meters
- pose order: `x,y,z,qx,qy,qz,qw`
- seed/reference joints: current joint state

The valid FK -> IK round-trip result is close to the current joint state.

## Verified Working Example

Current joints:

```text
[0.0197496, 0, 0, -0.000572205, -0.000572205, -0.000953674]
```

Current Cartesian pose:

```text
[-3.087220079578401e-07, 2.9270099233258406e-08, 0.2360610067844391,
 0.7068629860877991, 0.00751983979716897, 0.7072809934616089,
 -0.006444950122386217]
```

`forward_kine(tool_index=0, current_joints)` succeeds:

```text
forward_kine ret=1
pose=[-3.08722e-07, 2.92702e-08, 0.236061,
      0.706863, 0.00751983, 0.707281, -0.00644494]
```

`inverse_kine(tool_index=0, fk_current, current_joints)` succeeds:

```text
inverse_kine ret=1
solution=[0.0197496, 1.4521e-06, -1.17687e-06,
          -0.000571506, -0.000572503, -0.000947576]
```

## Convention Findings

Working:

- `cart_pose`
- `plan_pose`
- `fk_current`
- quaternion sign negation of the above
- `tool_index=0,1,2,3` all produced equivalent current-pose solutions in this test

Not working:

- `wxyz` payload order
- millimeter-scaled position payloads

Notes:

- Quaternion sign negation returns the same solution, as expected for equivalent quaternions.
- Quaternion conjugate returns a different valid-looking solution; it should not be used as the normal convention.
- Keep the normal project convention as `x,y,z,qx,qy,qz,qw` in meters.

## CLI Commands

Round-trip probe:

```bash
rosrun carm_a3_motion motion_cli.py ik-probe --include-mm
```

Small Cartesian offset solve only, no motion:

```bash
rosrun carm_a3_motion motion_cli.py ik-offset 0.01 0 0
```

#!/usr/bin/env python3
import argparse
import math
import sys

import rospy

from carm_a3_motion.srv import GetCartesianSnapshot
from carm_a3_motion.srv import GetJointSnapshot
from carm_a3_motion.srv import MoveJoint
from carm_a3_motion.srv import SolveIK


DEFAULT_TIMEOUT_S = 3.0


def get_param(name, default):
    return rospy.get_param("~" + name, default)


def service_proxy(name, service_type):
    try:
        rospy.wait_for_service(name, timeout=DEFAULT_TIMEOUT_S)
    except rospy.ROSException as exc:
        raise RuntimeError("service {} not available: {}".format(name, exc))
    return rospy.ServiceProxy(name, service_type)


def vector_to_text(values):
    return ",".join("{:.9g}".format(value) for value in values)


def max_abs_delta(a_values, b_values):
    return max(abs(a - b) for a, b in zip(a_values, b_values))


def interpolate_joints(start, target, max_step):
    if max_step <= 0.0:
        raise ValueError("segment_delta_rad must be > 0")
    max_delta = max_abs_delta(start, target)
    steps = max(1, int(math.ceil(max_delta / max_step)))
    waypoints = []
    for step in range(1, steps + 1):
        ratio = float(step) / float(steps)
        waypoints.append([
            a + (b - a) * ratio
            for a, b in zip(start, target)
        ])
    return waypoints


def normalize_quat_xyzw(quat):
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 1e-9:
        raise ValueError("target_quat_xyzw has near-zero norm")
    return [value / norm for value in quat]


def load_overview_pose(current_pose):
    target_xyz = get_param("overview/target_xyz_m", [0.30, 0.0, 0.36])
    if len(target_xyz) != 3:
        raise ValueError("overview/target_xyz_m must contain 3 values")

    use_current_orientation = bool(get_param("overview/use_current_orientation", True))
    if use_current_orientation:
        quat = list(current_pose[3:7])
    else:
        quat = get_param("overview/target_quat_xyzw", [0.707, 0.0, 0.707, 0.0])
    if len(quat) != 4:
        raise ValueError("overview quaternion must contain 4 values")
    quat = normalize_quat_xyzw([float(value) for value in quat])
    return [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2])] + quat


def validate_workspace_target(pose):
    region = get_param("workspace/table_region_m", {
        "x_min": 0.0,
        "x_max": 0.60,
        "y_min": -0.30,
        "y_max": 0.30,
    })
    x_min = float(region.get("x_min", 0.0))
    x_max = float(region.get("x_max", 0.60))
    y_min = float(region.get("y_min", -0.30))
    y_max = float(region.get("y_max", 0.30))
    table_z = float(get_param("workspace/table_z_m", 0.0))

    x, y, z = pose[:3]
    warnings = []
    if x < x_min or x > x_max:
        warnings.append("target x is outside configured table x range")
    if y < y_min or y > y_max:
        warnings.append("target y is outside configured table y range")
    if z <= table_z:
        warnings.append("target z is not above configured table_z_m")
    return warnings


def plan_overview():
    cart_proxy = service_proxy("/carm_a3/motion/get_cartesian_snapshot", GetCartesianSnapshot)
    joint_proxy = service_proxy("/carm_a3/motion/get_joint_snapshot", GetJointSnapshot)
    ik_proxy = service_proxy("/carm_a3/motion/solve_ik", SolveIK)

    cart_res = cart_proxy()
    print("cartesian snapshot:")
    print(cart_res)
    if not cart_res.success:
        return None, 1

    joint_res = joint_proxy()
    print("joint snapshot:")
    print(joint_res)
    if not joint_res.success:
        return None, 1

    target_pose = load_overview_pose(list(cart_res.pose))
    print("overview target pose x,y,z,qx,qy,qz,qw:")
    print(vector_to_text(target_pose))

    for warning in validate_workspace_target(target_pose):
        print("warning: {}".format(warning), file=sys.stderr)

    tool_index = int(get_param("overview/tool_index", 0))
    seed = list(joint_res.positions)
    ik_res = ik_proxy(tool_index, target_pose, seed)
    print("ik result:")
    print(ik_res)
    if not ik_res.success:
        return None, 1

    joint_delta = max_abs_delta(seed, ik_res.positions)
    print("max_joint_delta: {:.9g}".format(joint_delta))

    return {
        "target_pose": target_pose,
        "seed": seed,
        "target_joints": list(ik_res.positions),
        "max_joint_delta": joint_delta,
    }, 0


def main():
    parser = argparse.ArgumentParser(description="Plan or execute the CArm A3 pre-grasp overview pose")
    parser.add_argument("command", choices=["plan", "execute"])
    parser.add_argument("--max-joint-delta", type=float, default=None)
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--wait", action="store_true")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("carm_a3_grasp_init", anonymous=True)
    try:
        plan, code = plan_overview()
        if code != 0:
            return code
        max_delta = args.max_joint_delta
        if max_delta is None:
            max_delta = float(get_param("overview/max_joint_delta_rad", 0.35))
        if plan["max_joint_delta"] > max_delta:
            print(
                "blocked: max_joint_delta {:.9g} exceeds limit {:.9g}".format(
                    plan["max_joint_delta"], max_delta
                ),
                file=sys.stderr,
            )
            return 1

        print("move target joints:")
        print(vector_to_text(plan["target_joints"]))

        if args.command != "execute":
            print("not executing; rerun with 'execute' after checking the plan and camera clearance")
            return 0

        move_proxy = service_proxy("/carm_a3/motion/move_joint", MoveJoint)
        duration = args.duration_s
        if duration is None:
            duration = float(get_param("overview/duration_s", 3.0))
        wait = bool(args.wait or get_param("overview/wait", False))
        segment_delta = float(get_param("overview/segment_delta_rad", 0.10))
        waypoints = interpolate_joints(plan["seed"], plan["target_joints"], segment_delta)
        print("executing {} waypoint(s), segment_delta_rad={:.9g}".format(
            len(waypoints), segment_delta
        ))
        for index, waypoint in enumerate(waypoints, start=1):
            print("waypoint {}/{}: {}".format(index, len(waypoints), vector_to_text(waypoint)))
            move_res = move_proxy(waypoint, duration, wait)
            print(move_res)
            if not move_res.success:
                print("blocked: waypoint {} failed".format(index), file=sys.stderr)
                return 1
        return 0
    except (RuntimeError, rospy.ServiceException, ValueError) as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

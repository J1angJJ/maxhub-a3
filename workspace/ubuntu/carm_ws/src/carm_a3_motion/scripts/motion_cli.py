#!/usr/bin/env python3
import argparse
import sys

import rospy

from carm_a3_motion.srv import GetCartesianSnapshot
from carm_a3_motion.srv import GetJointSnapshot
from carm_a3_motion.srv import JogJoint
from carm_a3_motion.srv import MoveJoint
from carm_a3_motion.srv import SolveFK
from carm_a3_motion.srv import SolveIK


DEFAULT_TIMEOUT_S = 3.0


def vector_to_text(values):
    return ",".join("{:.9g}".format(value) for value in values)


def parse_float_list(text, expected=None):
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if expected is not None and len(values) != expected:
        raise argparse.ArgumentTypeError(
            "expected {} comma-separated values, got {}".format(expected, len(values))
        )
    return values


def max_abs_delta(a_values, b_values):
    return max(abs(a - b) for a, b in zip(a_values, b_values))


def parse_int_list(text):
    return [int(part.strip()) for part in text.split(",") if part.strip()]


def negate_quaternion(pose):
    values = list(pose)
    if len(values) == 7:
        values[3] = -values[3]
        values[4] = -values[4]
        values[5] = -values[5]
        values[6] = -values[6]
    return values


def conjugate_quaternion(pose):
    values = list(pose)
    if len(values) == 7:
        values[3] = -values[3]
        values[4] = -values[4]
        values[5] = -values[5]
    return values


def xyzw_to_wxyz_payload(pose):
    values = list(pose)
    if len(values) == 7:
        x, y, z, qx, qy, qz, qw = values
        return [x, y, z, qw, qx, qy, qz]
    return values


def scale_position(pose, scale):
    values = list(pose)
    if len(values) == 7:
        values[0] *= scale
        values[1] *= scale
        values[2] *= scale
    return values


def pose_variants(name, pose, include_mm=False):
    variants = [
        (name, list(pose)),
        (name + "_negq", negate_quaternion(pose)),
        (name + "_conj", conjugate_quaternion(pose)),
        (name + "_wxyz_payload", xyzw_to_wxyz_payload(pose)),
        (name + "_wxyz_payload_negq", negate_quaternion(xyzw_to_wxyz_payload(pose))),
    ]
    if include_mm:
        variants.extend([
            (variant_name + "_xyz_mm", scale_position(variant_pose, 1000.0))
            for variant_name, variant_pose in list(variants)
        ])
    return variants


def service_proxy(name, service_type):
    try:
        rospy.wait_for_service(name, timeout=DEFAULT_TIMEOUT_S)
    except rospy.ROSException as exc:
        raise RuntimeError("service {} not available: {}".format(name, exc))
    return rospy.ServiceProxy(name, service_type)


def call_snapshot(_args):
    proxy = service_proxy("/carm_a3/motion/get_joint_snapshot", GetJointSnapshot)
    res = proxy()
    print(res)
    return 0 if res.success else 1


def call_cart(_args):
    proxy = service_proxy("/carm_a3/motion/get_cartesian_snapshot", GetCartesianSnapshot)
    res = proxy()
    print(res)
    return 0 if res.success else 1


def call_jog(args):
    proxy = service_proxy("/carm_a3/motion/jog_joint", JogJoint)
    res = proxy(args.joint_index, args.delta_rad, args.duration_s)
    print(res)
    return 0 if res.success else 1


def call_move(args):
    proxy = service_proxy("/carm_a3/motion/move_joint", MoveJoint)
    res = proxy(parse_float_list(args.positions, 6), args.duration_s, args.wait)
    print(res)
    return 0 if res.success else 1


def call_ik(args):
    proxy = service_proxy("/carm_a3/motion/solve_ik", SolveIK)
    seed = parse_float_list(args.seed, 6) if args.seed else []
    res = proxy(args.tool_index, parse_float_list(args.pose, 7), seed)
    print(res)
    return 0 if res.success else 1


def call_ik_current(args):
    cart_proxy = service_proxy("/carm_a3/motion/get_cartesian_snapshot", GetCartesianSnapshot)
    ik_proxy = service_proxy("/carm_a3/motion/solve_ik", SolveIK)
    cart_res = cart_proxy()
    print(cart_res)
    if not cart_res.success:
        return 1
    seed = parse_float_list(args.seed, 6) if args.seed else []
    ik_res = ik_proxy(args.tool_index, cart_res.pose, seed)
    print(ik_res)
    return 0 if ik_res.success else 1


def call_ik_offset(args):
    cart_proxy = service_proxy("/carm_a3/motion/get_cartesian_snapshot", GetCartesianSnapshot)
    snapshot_proxy = service_proxy("/carm_a3/motion/get_joint_snapshot", GetJointSnapshot)
    ik_proxy = service_proxy("/carm_a3/motion/solve_ik", SolveIK)
    move_proxy = None
    if args.execute:
        move_proxy = service_proxy("/carm_a3/motion/move_joint", MoveJoint)

    cart_res = cart_proxy()
    print(cart_res)
    if not cart_res.success:
        return 1

    target_pose = list(cart_res.pose)
    target_pose[0] += args.dx
    target_pose[1] += args.dy
    target_pose[2] += args.dz

    if args.seed:
        seed = parse_float_list(args.seed, 6)
    else:
        snapshot_res = snapshot_proxy()
        print(snapshot_res)
        if not snapshot_res.success:
            return 1
        seed = list(snapshot_res.positions)

    print("target_pose: {}".format(target_pose))
    ik_res = ik_proxy(args.tool_index, target_pose, seed)
    print(ik_res)
    if not ik_res.success:
        return 1

    joint_delta = max_abs_delta(seed, ik_res.positions)
    print("max_joint_delta: {:.9g}".format(joint_delta))
    if joint_delta > args.max_joint_delta:
        print(
            "blocked: max_joint_delta {:.9g} exceeds limit {:.9g}".format(
                joint_delta, args.max_joint_delta
            ),
            file=sys.stderr,
        )
        return 1

    if not args.execute:
        print("not executing; rerun with --execute to call /carm_a3/motion/move_joint")
        print("move target: {}".format(vector_to_text(ik_res.positions)))
        return 0

    move_res = move_proxy(ik_res.positions, args.duration_s, args.wait)
    print(move_res)
    return 0 if move_res.success else 1


def call_ik_offset_scan(args):
    cart_proxy = service_proxy("/carm_a3/motion/get_cartesian_snapshot", GetCartesianSnapshot)
    snapshot_proxy = service_proxy("/carm_a3/motion/get_joint_snapshot", GetJointSnapshot)
    ik_proxy = service_proxy("/carm_a3/motion/solve_ik", SolveIK)

    cart_res = cart_proxy()
    print(cart_res)
    if not cart_res.success:
        return 1

    snapshot_res = snapshot_proxy()
    print(snapshot_res)
    if not snapshot_res.success:
        return 1
    seed = list(snapshot_res.positions)

    axis_index = {"x": 0, "y": 1, "z": 2}[args.axis]
    distances = parse_float_list(args.distances)
    any_success = False
    for distance in distances:
        target_pose = list(cart_res.pose)
        target_pose[axis_index] += distance
        ik_res = ik_proxy(args.tool_index, target_pose, seed)
        if ik_res.success:
            joint_delta = max_abs_delta(seed, ik_res.positions)
            print("axis={} distance={:.9g} success=true max_joint_delta={:.9g}".format(
                args.axis, distance, joint_delta
            ))
            print(ik_res)
            any_success = True
        else:
            print("axis={} distance={:.9g} success=false".format(args.axis, distance))
            print(ik_res)
    return 0 if any_success else 1


def call_ik_probe(args):
    snapshot_proxy = service_proxy("/carm_a3/motion/get_joint_snapshot", GetJointSnapshot)
    cart_proxy = service_proxy("/carm_a3/motion/get_cartesian_snapshot", GetCartesianSnapshot)
    fk_proxy = service_proxy("/carm_a3/motion/solve_fk", SolveFK)
    ik_proxy = service_proxy("/carm_a3/motion/solve_ik", SolveIK)

    snapshot_res = snapshot_proxy()
    print("joint snapshot:")
    print(snapshot_res)
    if not snapshot_res.success:
        return 1

    cart_res = cart_proxy()
    print("cartesian snapshot:")
    print(cart_res)
    if not cart_res.success:
        return 1

    seed = list(snapshot_res.positions)
    tool_indices = parse_int_list(args.tool_indices)
    any_success = False

    for tool_index in tool_indices:
        candidates = []
        candidates.extend(pose_variants("cart_pose", cart_res.pose, args.include_mm))
        candidates.extend(pose_variants("plan_pose", cart_res.plan_pose, args.include_mm))
        fk_res = fk_proxy(tool_index, seed)
        print("fk_current tool_index={}:".format(tool_index))
        print(fk_res)
        if fk_res.success:
            candidates.extend(pose_variants("fk_current", fk_res.pose, args.include_mm))

        for name, pose in candidates:
            ik_res = ik_proxy(tool_index, pose, seed)
            print("ik_probe tool_index={} candidate={} success={}".format(
                tool_index, name, ik_res.success
            ))
            print(ik_res)
            any_success = any_success or ik_res.success

    return 0 if any_success else 1


def call_fk(args):
    proxy = service_proxy("/carm_a3/motion/solve_fk", SolveFK)
    res = proxy(args.tool_index, parse_float_list(args.positions, 6))
    print(res)
    return 0 if res.success else 1


def main():
    parser = argparse.ArgumentParser(description="CArm A3 ROS motion service helper")
    subparsers = parser.add_subparsers(dest="command", required=True)

    snapshot = subparsers.add_parser("snapshot")
    snapshot.set_defaults(func=call_snapshot)

    cart = subparsers.add_parser("cart")
    cart.set_defaults(func=call_cart)

    jog = subparsers.add_parser("jog")
    jog.add_argument("joint_index", type=int)
    jog.add_argument("delta_rad", type=float)
    jog.add_argument("--duration-s", type=float, default=2.0)
    jog.set_defaults(func=call_jog)

    move = subparsers.add_parser("move")
    move.add_argument("positions", help="six comma-separated joint values in rad")
    move.add_argument("--duration-s", type=float, default=2.0)
    move.add_argument("--wait", action="store_true")
    move.set_defaults(func=call_move)

    ik = subparsers.add_parser("ik")
    ik.add_argument("pose", help="x,y,z,qx,qy,qz,qw")
    ik.add_argument("--tool-index", type=int, default=0)
    ik.add_argument("--seed", default="", help="optional six comma-separated seed joints")
    ik.set_defaults(func=call_ik)

    ik_current = subparsers.add_parser("ik-current")
    ik_current.add_argument("--tool-index", type=int, default=0)
    ik_current.add_argument("--seed", default="", help="optional six comma-separated seed joints")
    ik_current.set_defaults(func=call_ik_current)

    ik_offset = subparsers.add_parser("ik-offset")
    ik_offset.add_argument("dx", type=float, help="x offset in meters")
    ik_offset.add_argument("dy", type=float, help="y offset in meters")
    ik_offset.add_argument("dz", type=float, help="z offset in meters")
    ik_offset.add_argument("--tool-index", type=int, default=0)
    ik_offset.add_argument("--seed", default="", help="optional six comma-separated seed joints")
    ik_offset.add_argument("--max-joint-delta", type=float, default=0.05)
    ik_offset.add_argument("--duration-s", type=float, default=2.0)
    ik_offset.add_argument("--wait", action="store_true")
    ik_offset.add_argument("--execute", action="store_true")
    ik_offset.set_defaults(func=call_ik_offset)

    ik_offset_scan = subparsers.add_parser("ik-offset-scan")
    ik_offset_scan.add_argument("axis", choices=["x", "y", "z"])
    ik_offset_scan.add_argument(
        "--distances",
        default="-0.001,-0.002,-0.005,-0.01,0.001,0.002,0.005,0.01",
        help="comma-separated offsets in meters",
    )
    ik_offset_scan.add_argument("--tool-index", type=int, default=0)
    ik_offset_scan.set_defaults(func=call_ik_offset_scan)

    ik_probe = subparsers.add_parser("ik-probe")
    ik_probe.add_argument("--tool-indices", default="0,1,2,3")
    ik_probe.add_argument("--include-mm", action="store_true")
    ik_probe.set_defaults(func=call_ik_probe)

    fk = subparsers.add_parser("fk")
    fk.add_argument("positions", help="six comma-separated joint values in rad")
    fk.add_argument("--tool-index", type=int, default=0)
    fk.set_defaults(func=call_fk)

    args = parser.parse_args()
    rospy.init_node("carm_a3_motion_cli", anonymous=True)
    try:
        return args.func(args)
    except (RuntimeError, rospy.ServiceException) as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

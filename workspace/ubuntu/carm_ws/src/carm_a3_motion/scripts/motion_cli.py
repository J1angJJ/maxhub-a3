#!/usr/bin/env python3
import argparse
import sys

import rospy

from carm_a3_motion.srv import GetCartesianSnapshot
from carm_a3_motion.srv import GetExtendedState
from carm_a3_motion.srv import GetJointSnapshot
from carm_a3_motion.srv import GetToolInfo
from carm_a3_motion.srv import JogJoint
from carm_a3_motion.srv import MoveFlowPose
from carm_a3_motion.srv import MoveJoint
from carm_a3_motion.srv import MoveLineJoint
from carm_a3_motion.srv import MoveLinePose
from carm_a3_motion.srv import MovePose
from carm_a3_motion.srv import SetCollisionConfig
from carm_a3_motion.srv import SetControlMode
from carm_a3_motion.srv import SetGripper
from carm_a3_motion.srv import SetSpeed
from carm_a3_motion.srv import SetToolIndex
from carm_a3_motion.srv import SolveFK
from carm_a3_motion.srv import SolveFKArray
from carm_a3_motion.srv import SolveIK
from carm_a3_motion.srv import SolveIKArray


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


def call_extended(_args):
    proxy = service_proxy("/carm_a3/motion/get_extended_state", GetExtendedState)
    res = proxy()
    print(res)
    return 0 if res.success else 1


def call_tool_info(args):
    proxy = service_proxy("/carm_a3/motion/get_tool_info", GetToolInfo)
    res = proxy(args.index)
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


def call_move_pose(args):
    proxy = service_proxy("/carm_a3/motion/move_pose", MovePose)
    res = proxy(parse_float_list(args.pose, 7), args.duration_s, args.wait)
    print(res)
    return 0 if res.success else 1


def call_move_line_joint(args):
    proxy = service_proxy("/carm_a3/motion/move_line_joint", MoveLineJoint)
    res = proxy(parse_float_list(args.positions, 6), args.wait)
    print(res)
    return 0 if res.success else 1


def call_move_line_pose(args):
    proxy = service_proxy("/carm_a3/motion/move_line_pose", MoveLinePose)
    res = proxy(parse_float_list(args.pose, 7), args.wait)
    print(res)
    return 0 if res.success else 1


def call_move_flow_pose(args):
    proxy = service_proxy("/carm_a3/motion/move_flow_pose", MoveFlowPose)
    res = proxy(parse_float_list(args.pose, 7), args.line_theta_weight, args.accuracy, args.wait)
    print(res)
    return 0 if res.success else 1


def call_set_speed(args):
    proxy = service_proxy("/carm_a3/motion/set_speed", SetSpeed)
    res = proxy(args.level, args.response_level)
    print(res)
    return 0 if res.success else 1


def call_set_collision(args):
    proxy = service_proxy("/carm_a3/motion/set_collision_config", SetCollisionConfig)
    res = proxy(args.enable, args.sensitivity_level)
    print(res)
    return 0 if res.success else 1


def call_set_control_mode(args):
    proxy = service_proxy("/carm_a3/motion/set_control_mode", SetControlMode)
    res = proxy(args.mode)
    print(res)
    return 0 if res.success else 1


def call_set_tool(args):
    proxy = service_proxy("/carm_a3/motion/set_tool_index", SetToolIndex)
    res = proxy(args.index)
    print(res)
    return 0 if res.success else 1


def call_set_gripper(args):
    proxy = service_proxy("/carm_a3/motion/set_gripper", SetGripper)
    res = proxy(args.pos, args.tau)
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
    ik_array_proxy = service_proxy("/carm_a3/motion/solve_ik_array", SolveIKArray)

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
    poses = []
    seeds = []
    for distance in distances:
        target_pose = list(cart_res.pose)
        target_pose[axis_index] += distance
        poses.extend(target_pose)
        seeds.extend(seed)

    ik_array_res = ik_array_proxy(args.tool_index, poses, seeds)
    print(ik_array_res.message)

    any_success = False
    for i, distance in enumerate(distances):
        start = i * 6
        end = start + 6
        positions = list(ik_array_res.positions[start:end])
        point_success = i < len(ik_array_res.point_success) and ik_array_res.point_success[i]
        if point_success:
            joint_delta = max_abs_delta(seed, positions)
            print("axis={} distance={:.9g} success=true max_joint_delta={:.9g}".format(
                args.axis, distance, joint_delta
            ))
            print("positions: [{}]".format(vector_to_text(positions)))
            any_success = True
        else:
            print("axis={} distance={:.9g} success=false".format(args.axis, distance))
    return 0 if any_success else 1


def call_fk_array(args):
    proxy = service_proxy("/carm_a3/motion/solve_fk_array", SolveFKArray)
    positions = parse_float_list(args.positions)
    res = proxy(args.tool_index, positions)
    print(res)
    return 0 if res.success else 1


def call_ik_array(args):
    proxy = service_proxy("/carm_a3/motion/solve_ik_array", SolveIKArray)
    poses = parse_float_list(args.poses)
    seeds = parse_float_list(args.seed_positions) if args.seed_positions else []
    res = proxy(args.tool_index, poses, seeds)
    print(res)
    return 0 if res.success else 1


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

    extended = subparsers.add_parser("extended")
    extended.set_defaults(func=call_extended)

    tool_info = subparsers.add_parser("tool-info")
    tool_info.add_argument("index", type=int)
    tool_info.set_defaults(func=call_tool_info)

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

    move_pose = subparsers.add_parser("move-pose")
    move_pose.add_argument("pose", help="x,y,z,qx,qy,qz,qw")
    move_pose.add_argument("--duration-s", type=float, default=2.0)
    move_pose.add_argument("--wait", action="store_true")
    move_pose.set_defaults(func=call_move_pose)

    move_line_joint = subparsers.add_parser("move-line-joint")
    move_line_joint.add_argument("positions", help="six comma-separated joint values in rad")
    move_line_joint.add_argument("--wait", action="store_true")
    move_line_joint.set_defaults(func=call_move_line_joint)

    move_line_pose = subparsers.add_parser("move-line-pose")
    move_line_pose.add_argument("pose", help="x,y,z,qx,qy,qz,qw")
    move_line_pose.add_argument("--wait", action="store_true")
    move_line_pose.set_defaults(func=call_move_line_pose)

    move_flow_pose = subparsers.add_parser("move-flow-pose")
    move_flow_pose.add_argument("pose", help="x,y,z,qx,qy,qz,qw")
    move_flow_pose.add_argument("--line-theta-weight", type=float, default=0.5)
    move_flow_pose.add_argument("--accuracy", type=float, default=0.0001)
    move_flow_pose.add_argument("--wait", action="store_true")
    move_flow_pose.set_defaults(func=call_move_flow_pose)

    set_speed = subparsers.add_parser("set-speed")
    set_speed.add_argument("level", type=float)
    set_speed.add_argument("--response-level", type=int, default=20)
    set_speed.set_defaults(func=call_set_speed)

    set_collision = subparsers.add_parser("set-collision")
    set_collision.add_argument("enable", type=lambda value: value.lower() in ["1", "true", "yes", "on"])
    set_collision.add_argument("--sensitivity-level", type=int, default=0)
    set_collision.set_defaults(func=call_set_collision)

    set_control_mode = subparsers.add_parser("set-control-mode")
    set_control_mode.add_argument("mode", type=int)
    set_control_mode.set_defaults(func=call_set_control_mode)

    set_tool = subparsers.add_parser("set-tool")
    set_tool.add_argument("index", type=int)
    set_tool.set_defaults(func=call_set_tool)

    set_gripper = subparsers.add_parser("set-gripper")
    set_gripper.add_argument("pos", type=float, help="finger gap in meters")
    set_gripper.add_argument("--tau", type=float, default=10.0)
    set_gripper.set_defaults(func=call_set_gripper)

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

    fk_array = subparsers.add_parser("fk-array")
    fk_array.add_argument("positions", help="flattened joint values, 6 values per point")
    fk_array.add_argument("--tool-index", type=int, default=0)
    fk_array.set_defaults(func=call_fk_array)

    ik_array = subparsers.add_parser("ik-array")
    ik_array.add_argument("poses", help="flattened poses, 7 values per point")
    ik_array.add_argument("--seed-positions", default="", help="optional flattened seeds, 6 values per point")
    ik_array.add_argument("--tool-index", type=int, default=0)
    ik_array.set_defaults(func=call_ik_array)

    args = parser.parse_args()
    rospy.init_node("carm_a3_motion_cli", anonymous=True)
    try:
        return args.func(args)
    except (RuntimeError, rospy.ServiceException) as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

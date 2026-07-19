#!/usr/bin/env python3
import argparse
import sys

import rospy

from carm_a3_motion.srv import GetJointSnapshot
from carm_a3_motion.srv import JogJoint
from carm_a3_motion.srv import MoveJoint
from carm_a3_motion.srv import SolveFK
from carm_a3_motion.srv import SolveIK


DEFAULT_TIMEOUT_S = 3.0


def parse_float_list(text, expected=None):
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if expected is not None and len(values) != expected:
        raise argparse.ArgumentTypeError(
            "expected {} comma-separated values, got {}".format(expected, len(values))
        )
    return values


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

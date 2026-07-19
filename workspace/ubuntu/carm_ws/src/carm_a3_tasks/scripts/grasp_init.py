#!/usr/bin/env python3
import argparse
import math
import sys

import rospy
import roslib
import yaml
import numpy as np
from tf.transformations import quaternion_from_matrix, quaternion_matrix

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


def resolve_path(path):
    if path.startswith("$(find "):
        end = path.find(")")
        if end > 0:
            package = path[len("$(find "):end]
            suffix = path[end + 1:].lstrip("/\\")
            return roslib.packages.get_pkg_dir(package) + "/" + suffix
    return path


def load_yaml(path):
    with open(resolve_path(path), "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def quat_to_matrix(quat_xyzw):
    return np.asarray(quaternion_matrix(normalize_quat_xyzw(quat_xyzw))[:3, :3], dtype=float)


def matrix_to_quat_xyzw(rotation):
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, dtype=float)
    quat = quaternion_from_matrix(matrix)
    return normalize_quat_xyzw([float(v) for v in quat])


def mat_vec_mul(matrix, vector):
    return list(np.asarray(matrix, dtype=float).dot(np.asarray(vector, dtype=float)))


def mat_mul(a_matrix, b_matrix):
    return np.asarray(a_matrix, dtype=float).dot(np.asarray(b_matrix, dtype=float))


def mat_transpose(matrix):
    return np.asarray(matrix, dtype=float).T


def vec_sub(a_values, b_values):
    return [a - b for a, b in zip(a_values, b_values)]


def load_camera_model():
    path = get_param(
        "camera_check/camera_info_yaml",
        "$(find carm_a3_vision)/config/camera_info.yaml",
    )
    data = load_yaml(path)
    width = float(data["image_width"])
    height = float(data["image_height"])
    k = data["camera_matrix"]["data"]
    fx = float(k[0])
    fy = float(k[4])
    hfov = 2.0 * math.atan(width / (2.0 * fx))
    vfov = 2.0 * math.atan(height / (2.0 * fy))
    return {
        "width": width,
        "height": height,
        "fx": fx,
        "fy": fy,
        "hfov": hfov,
        "vfov": vfov,
    }


def load_handeye():
    path = get_param(
        "camera_check/handeye_yaml",
        "$(find carm_a3_calibration)/config/handeye_flange_camera.yaml",
    )
    data = load_yaml(path)
    t = data["translation"]
    q = data["rotation"]
    translation = [float(t["x"]), float(t["y"]), float(t["z"])]
    rotation = quat_to_matrix([float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])])
    return translation, rotation


def table_region():
    region = get_param("workspace/table_region_m", {
        "x_min": 0.0,
        "x_max": 0.60,
        "y_min": -0.30,
        "y_max": 0.30,
    })
    return {
        "x_min": float(region.get("x_min", 0.0)),
        "x_max": float(region.get("x_max", 0.60)),
        "y_min": float(region.get("y_min", -0.30)),
        "y_max": float(region.get("y_max", 0.30)),
    }


def camera_down_rotation():
    # Camera optical frame: +Z forward, +X right, +Y down.
    # For table overview, optical +Z points toward base -Z.
    x_cam = [0.0, -1.0, 0.0]
    y_cam = [-1.0, 0.0, 0.0]
    z_cam = [0.0, 0.0, -1.0]
    return [
        [x_cam[0], y_cam[0], z_cam[0]],
        [x_cam[1], y_cam[1], z_cam[1]],
        [x_cam[2], y_cam[2], z_cam[2]],
    ]


def solve_fov_overview_pose():
    camera = load_camera_model()
    flange_t_camera, flange_r_camera = load_handeye()
    region = table_region()
    table_z = float(get_param("workspace/table_z_m", 0.0))
    margin = float(get_param("overview/coverage_margin_m", 0.08))
    min_height = float(get_param("overview/min_camera_height_m", 0.25))
    max_height = float(get_param("overview/max_camera_height_m", 0.65))

    table_x = region["x_max"] - region["x_min"]
    table_y = region["y_max"] - region["y_min"]
    need_x = table_x + 2.0 * margin
    need_y = table_y + 2.0 * margin

    # With the chosen camera-down yaw, image horizontal spans table Y and image vertical spans table X.
    height_for_y = need_y / (2.0 * math.tan(camera["hfov"] / 2.0))
    height_for_x = need_x / (2.0 * math.tan(camera["vfov"] / 2.0))
    camera_height = max(height_for_x, height_for_y, min_height)
    if camera_height > max_height:
        print(
            "warning: required camera height {:.3f} m exceeds configured max {:.3f} m".format(
                camera_height, max_height
            ),
            file=sys.stderr,
        )
        camera_height = max_height

    camera_center = [
        (region["x_min"] + region["x_max"]) * 0.5,
        (region["y_min"] + region["y_max"]) * 0.5,
        table_z + camera_height,
    ]

    base_r_camera = camera_down_rotation()
    base_r_flange = mat_mul(base_r_camera, mat_transpose(flange_r_camera))
    base_t_flange = vec_sub(camera_center, mat_vec_mul(base_r_flange, flange_t_camera))
    quat = matrix_to_quat_xyzw(base_r_flange)

    covered_y = 2.0 * camera_height * math.tan(camera["hfov"] / 2.0)
    covered_x = 2.0 * camera_height * math.tan(camera["vfov"] / 2.0)
    return {
        "pose": base_t_flange + quat,
        "camera_center": camera_center,
        "camera_height": camera_height,
        "hfov_deg": math.degrees(camera["hfov"]),
        "vfov_deg": math.degrees(camera["vfov"]),
        "covered_x_m": covered_x,
        "covered_y_m": covered_y,
        "required_x_m": need_x,
        "required_y_m": need_y,
    }


def load_overview_pose(current_pose):
    if bool(get_param("overview/use_fov_solver", True)):
        solution = solve_fov_overview_pose()
        print("fov overview:")
        for key in ["hfov_deg", "vfov_deg", "camera_height", "covered_x_m", "covered_y_m",
                    "required_x_m", "required_y_m"]:
            print("{}: {:.6g}".format(key, solution[key]))
        print("camera_center: {}".format(vector_to_text(solution["camera_center"])))
        return solution["pose"]

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


def print_fov_only():
    solution = solve_fov_overview_pose()
    print("fov overview target flange pose x,y,z,qx,qy,qz,qw:")
    print(vector_to_text(solution["pose"]))
    print("camera_center:")
    print(vector_to_text(solution["camera_center"]))
    for key in ["hfov_deg", "vfov_deg", "camera_height", "covered_x_m", "covered_y_m",
                "required_x_m", "required_y_m"]:
        print("{}: {:.9g}".format(key, solution[key]))


def main():
    parser = argparse.ArgumentParser(description="Plan or execute the CArm A3 pre-grasp overview pose")
    parser.add_argument("command", choices=["fov", "plan", "execute"])
    parser.add_argument("--max-joint-delta", type=float, default=None)
    parser.add_argument("--duration-s", type=float, default=None)
    parser.add_argument("--wait", action="store_true")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])

    rospy.init_node("carm_a3_grasp_init")
    try:
        if args.command == "fov":
            print_fov_only()
            return 0
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

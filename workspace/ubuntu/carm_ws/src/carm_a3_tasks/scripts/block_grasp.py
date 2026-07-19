#!/usr/bin/env python3
import argparse
import json
import math
import sys

import numpy as np
import rospy
import tf
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from carm_a3_motion.srv import GetCartesianSnapshot
from carm_a3_motion.srv import GetJointSnapshot
from carm_a3_motion.srv import MoveJoint
from carm_a3_motion.srv import SetGripper
from carm_a3_motion.srv import SolveIK


DEFAULT_TIMEOUT_S = 5.0


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


def normalize_quat_xyzw(quat):
    norm = math.sqrt(sum(value * value for value in quat))
    if norm <= 1e-9:
        raise ValueError("quaternion has near-zero norm")
    return [value / norm for value in quat]


def quat_to_matrix(quat_xyzw):
    return np.asarray(tf.transformations.quaternion_matrix(normalize_quat_xyzw(quat_xyzw))[:3, :3], dtype=float)


def matrix_to_quat_xyzw(rotation):
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(rotation, dtype=float)
    quat = tf.transformations.quaternion_from_matrix(matrix)
    return normalize_quat_xyzw([float(value) for value in quat])


def yaw_rotation(delta_rad):
    cos_v = math.cos(delta_rad)
    sin_v = math.sin(delta_rad)
    return np.asarray([
        [cos_v, -sin_v, 0.0],
        [sin_v, cos_v, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=float)


def signed_angle_xy(from_vec, to_vec):
    from_xy = np.asarray([from_vec[0], from_vec[1]], dtype=float)
    to_xy = np.asarray([to_vec[0], to_vec[1]], dtype=float)
    from_norm = np.linalg.norm(from_xy)
    to_norm = np.linalg.norm(to_xy)
    if from_norm <= 1e-9 or to_norm <= 1e-9:
        raise ValueError("cannot compute yaw from near-zero XY vector")
    from_xy = from_xy / from_norm
    to_xy = to_xy / to_norm
    cross = from_xy[0] * to_xy[1] - from_xy[1] * to_xy[0]
    dot = float(np.dot(from_xy, to_xy))
    return math.atan2(cross, dot)


def interpolate_joints(start, target, max_step):
    if max_step <= 0.0:
        raise ValueError("segment_delta_rad must be > 0")
    max_delta = max_abs_delta(start, target)
    steps = max(1, int(math.ceil(max_delta / max_step)))
    waypoints = []
    for step in range(1, steps + 1):
        ratio = float(step) / float(steps)
        waypoints.append([a + (b - a) * ratio for a, b in zip(start, target)])
    return waypoints


def camera_model_from_info(info):
    if len(info.K) < 9 or abs(info.K[0]) < 1e-9 or abs(info.K[4]) < 1e-9:
        raise ValueError("camera_info has invalid K")
    return {
        "fx": float(info.K[0]),
        "fy": float(info.K[4]),
        "cx": float(info.K[2]),
        "cy": float(info.K[5]),
        "frame_id": info.header.frame_id,
    }


def wait_json(topic):
    msg = rospy.wait_for_message(topic, String, timeout=DEFAULT_TIMEOUT_S)
    return json.loads(msg.data)


def wait_camera_info(topic):
    msg = rospy.wait_for_message(topic, CameraInfo, timeout=DEFAULT_TIMEOUT_S)
    return camera_model_from_info(msg)


def choose_detection(payload, target_color, min_confidence):
    detections = payload.get("detections", [])
    candidates = []
    for det in detections:
        if target_color and det.get("color") != target_color:
            continue
        if float(det.get("confidence", 0.0)) < min_confidence:
            continue
        candidates.append(det)
    candidates.sort(key=lambda item: (
        float(item.get("confidence", 0.0)),
        float(item.get("area_px", 0.0)),
    ), reverse=True)
    if not candidates:
        raise ValueError("no matching color block detection")
    return candidates[0]


def transform_ray_to_base(listener, base_frame, camera_frame, stamp, ray_camera):
    try:
        listener.waitForTransform(base_frame, camera_frame, stamp, rospy.Duration(DEFAULT_TIMEOUT_S))
        translation, rotation = listener.lookupTransform(base_frame, camera_frame, stamp)
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        listener.waitForTransform(base_frame, camera_frame, rospy.Time(0), rospy.Duration(DEFAULT_TIMEOUT_S))
        translation, rotation = listener.lookupTransform(base_frame, camera_frame, rospy.Time(0))

    matrix = tf.transformations.quaternion_matrix(rotation)
    origin = np.asarray(translation, dtype=float)
    direction = np.asarray(matrix[:3, :3], dtype=float).dot(np.asarray(ray_camera, dtype=float))
    norm = np.linalg.norm(direction)
    if norm <= 1e-9:
        raise ValueError("projected camera ray has near-zero norm")
    return origin, direction / norm


def payload_stamp_and_frame(payload, camera_model):
    header = payload.get("header", {})
    stamp_data = header.get("stamp", {})
    stamp = rospy.Time(int(stamp_data.get("secs", 0)), int(stamp_data.get("nsecs", 0)))
    camera_frame = header.get("frame_id") or camera_model["frame_id"] or get_param(
        "perception/camera_frame_id", "carm_a3_camera_optical_frame"
    )
    return stamp, camera_frame


def project_pixel_to_plane(listener, camera_model, payload, pixel, projection_z):
    base_frame = get_param("workspace/base_frame_id", "base_link")

    u, v = pixel
    ray_camera = [
        (float(u) - camera_model["cx"]) / camera_model["fx"],
        (float(v) - camera_model["cy"]) / camera_model["fy"],
        1.0,
    ]

    stamp, camera_frame = payload_stamp_and_frame(payload, camera_model)

    origin, direction = transform_ray_to_base(listener, base_frame, camera_frame, stamp, ray_camera)
    if abs(direction[2]) <= 1e-9:
        raise ValueError("camera ray is parallel to projection plane")
    scale = (projection_z - origin[2]) / direction[2]
    if scale <= 0.0:
        raise ValueError("projection plane is behind camera")
    point = origin + direction * scale
    return np.asarray([float(point[0]), float(point[1]), float(projection_z)], dtype=float)


def project_detection_to_table(listener, detection, camera_model, payload):
    table_z = float(get_param("workspace/table_z_m", 0.0))
    block_size = get_param("block/size_m", [0.05, 0.05, 0.10])
    block_height = float(block_size[2])
    projection_z = table_z + block_height * 0.5
    point = project_pixel_to_plane(listener, camera_model, payload, detection["center_px"], projection_z)
    return [float(point[0]), float(point[1]), float(point[2])]


def estimate_block_long_edge(listener, detection, camera_model, payload):
    corners = detection.get("corners_px", [])
    if len(corners) != 4:
        return None
    table_z = float(get_param("workspace/table_z_m", 0.0))
    block_size = get_param("block/size_m", [0.05, 0.05, 0.10])
    projection_z = table_z + float(block_size[2]) * 0.5
    points = [
        project_pixel_to_plane(listener, camera_model, payload, corner, projection_z)
        for corner in corners
    ]
    best_vec = None
    best_len = 0.0
    for index in range(4):
        vec = points[(index + 1) % 4] - points[index]
        vec[2] = 0.0
        length = float(np.linalg.norm(vec))
        if length > best_len:
            best_len = length
            best_vec = vec
    if best_vec is None or best_len <= 1e-9:
        return None
    best_vec = best_vec / best_len
    if best_vec[0] < 0.0:
        best_vec = -best_vec
    yaw_deg = math.degrees(math.atan2(best_vec[1], best_vec[0]))
    return best_vec, best_len, yaw_deg


def align_quat_to_block_long_edge(quat, long_edge):
    if long_edge is None or not bool(get_param("grasp/align_to_block_long_edge", True)):
        return quat
    axis_name = str(get_param("grasp/align_tool_axis", "y")).lower()
    if axis_name == "x":
        local_axis = np.asarray([1.0, 0.0, 0.0], dtype=float)
    elif axis_name == "neg_x":
        local_axis = np.asarray([-1.0, 0.0, 0.0], dtype=float)
    elif axis_name == "neg_y":
        local_axis = np.asarray([0.0, -1.0, 0.0], dtype=float)
    else:
        local_axis = np.asarray([0.0, 1.0, 0.0], dtype=float)

    rotation = quat_to_matrix(quat)
    current_axis = rotation.dot(local_axis)
    delta = signed_angle_xy(current_axis, long_edge)
    delta += math.radians(float(get_param("grasp/align_yaw_offset_deg", 0.0)))
    aligned = yaw_rotation(delta).dot(rotation)
    print(
        "orientation alignment: axis={}, yaw_delta_deg={:.6g}".format(
            axis_name, math.degrees(delta)
        )
    )
    return matrix_to_quat_xyzw(aligned)


def build_grasp_poses(point, current_pose, long_edge):
    use_current_orientation = bool(get_param("grasp/use_current_orientation", True))
    if use_current_orientation:
        quat = list(current_pose[3:7])
    else:
        quat = get_param("grasp/target_quat_xyzw", [0.707, 0.0, 0.707, 0.0])
    quat = normalize_quat_xyzw([float(value) for value in quat])
    quat = align_quat_to_block_long_edge(quat, long_edge)

    approach_height = float(get_param("grasp/approach_height_m", 0.12))
    grasp_z = float(get_param("grasp/grasp_z_m", 0.09))
    lift_height = float(get_param("grasp/lift_height_m", 0.10))

    x, y, _ = point
    grasp = [x, y, grasp_z] + quat
    approach = [x, y, grasp_z + approach_height] + quat
    lift = [x, y, grasp_z + lift_height] + quat
    return {
        "approach": approach,
        "grasp": grasp,
        "lift": lift,
    }


def validate_pose_heights(poses, names):
    min_z = float(get_param("grasp/min_flange_z_m", 0.18))
    for name in names:
        pose = poses[name]
        if float(pose[2]) < min_z:
            raise RuntimeError(
                "{} flange z {:.6g} is below min_flange_z_m {:.6g}; refusing to plan near table".format(
                    name, float(pose[2]), min_z
                )
            )


def solve_pose_sequence(ik_proxy, poses, seed, names):
    tool_index = int(get_param("grasp/tool_index", 0))
    solved = []
    current_seed = list(seed)
    for name in names:
        res = ik_proxy(tool_index, poses[name], current_seed)
        print("{} ik result:".format(name))
        print(res)
        if not res.success:
            raise RuntimeError("{} IK failed: {}".format(name, res.message))
        positions = list(res.positions)
        solved.append((name, poses[name], positions, max_abs_delta(current_seed, positions)))
        current_seed = positions
    return solved


def execute_sequence(move_proxy, solved):
    duration = float(get_param("grasp/duration_s", 2.0))
    wait = bool(get_param("grasp/wait", False))
    segment_delta = float(get_param("grasp/segment_delta_rad", 0.08))
    current = solved[0][2]
    # The first element's interpolation start is filled by the caller.
    del current
    for name, _, target, _ in solved:
        print("executing {} target: {}".format(name, vector_to_text(target)))
        if not hasattr(execute_sequence, "last_positions"):
            raise RuntimeError("execute_sequence.last_positions is not set")
        waypoints = interpolate_joints(execute_sequence.last_positions, target, segment_delta)
        for index, waypoint in enumerate(waypoints, start=1):
            print("{} waypoint {}/{}: {}".format(name, index, len(waypoints), vector_to_text(waypoint)))
            res = move_proxy(waypoint, duration, wait)
            print(res)
            if not res.success:
                raise RuntimeError("{} waypoint {} failed: {}".format(name, index, res.message))
        execute_sequence.last_positions = target


def maybe_set_gripper(pos, label):
    proxy = service_proxy("/carm_a3/motion/set_gripper", SetGripper)
    tau = float(get_param("grasp/gripper_tau_n", 5.0))
    print("setting gripper {}: pos={:.6g}, tau={:.6g}".format(label, pos, tau))
    res = proxy(float(pos), tau)
    print(res)
    if not res.success:
        raise RuntimeError("set_gripper {} failed: {}".format(label, res.message))


def plan_block_grasp(args):
    detection_topic = get_param("perception/detections_topic", "/carm_a3/perception/color_blocks")
    camera_info_topic = get_param("perception/camera_info_topic", "/carm_a3/camera/camera_info")
    target_color = args.color if args.color is not None else get_param("perception/target_color", "")
    min_confidence = float(get_param("perception/min_confidence", 0.45))

    payload = wait_json(detection_topic)
    camera_model = wait_camera_info(camera_info_topic)
    detection = choose_detection(payload, target_color, min_confidence)
    print("selected detection:")
    print(json.dumps(detection, sort_keys=True))

    listener = tf.TransformListener()
    rospy.sleep(0.2)
    point = project_detection_to_table(listener, detection, camera_model, payload)
    print("estimated block center in base frame x,y,z:")
    print(vector_to_text(point))
    long_edge_info = estimate_block_long_edge(listener, detection, camera_model, payload)
    long_edge = None
    if long_edge_info is not None:
        long_edge, long_edge_len, long_edge_yaw = long_edge_info
        print("estimated block long edge in base frame:")
        print("direction={}, length_m={:.9g}, yaw_deg={:.6g}".format(
            vector_to_text(long_edge), long_edge_len, long_edge_yaw
        ))

    cart_proxy = service_proxy("/carm_a3/motion/get_cartesian_snapshot", GetCartesianSnapshot)
    joint_proxy = service_proxy("/carm_a3/motion/get_joint_snapshot", GetJointSnapshot)
    ik_proxy = service_proxy("/carm_a3/motion/solve_ik", SolveIK)

    cart_res = cart_proxy()
    print("cartesian snapshot:")
    print(cart_res)
    if not cart_res.success:
        raise RuntimeError(cart_res.message)
    joint_res = joint_proxy()
    print("joint snapshot:")
    print(joint_res)
    if not joint_res.success:
        raise RuntimeError(joint_res.message)

    poses = build_grasp_poses(point, list(cart_res.pose), long_edge)
    print("planned poses x,y,z,qx,qy,qz,qw:")
    for name in ["approach", "grasp", "lift"]:
        print("{}: {}".format(name, vector_to_text(poses[name])))

    sequence_names = ["approach", "grasp", "lift"] if args.allow_descend else ["approach"]
    if not args.allow_descend:
        print("safe planning mode: planning approach only; add --allow-descend to include grasp/lift")
    validate_pose_heights(poses, sequence_names)

    solved = solve_pose_sequence(ik_proxy, poses, list(joint_res.positions), sequence_names)
    max_total = float(get_param("grasp/max_total_joint_delta_rad", 2.20))
    first_delta = max_abs_delta(list(joint_res.positions), solved[0][2])
    max_segment = max([first_delta] + [item[3] for item in solved[1:]])
    print("max_sequence_joint_delta: {:.9g}".format(max_segment))
    if max_segment > max_total:
        raise RuntimeError(
            "planned joint delta {:.9g} exceeds limit {:.9g}".format(max_segment, max_total)
        )

    print("joint targets:")
    for name, _, joints, delta in solved:
        print("{} delta={:.9g}: {}".format(name, delta, vector_to_text(joints)))

    if not args.execute:
        print("not executing; rerun with --execute after checking target and clearance")
        return

    move_proxy = service_proxy("/carm_a3/motion/move_joint", MoveJoint)
    execute_sequence.last_positions = list(joint_res.positions)
    if bool(get_param("grasp/open_before_approach", True)):
        maybe_set_gripper(float(get_param("grasp/open_gripper_pos_m", 0.065)), "open-before-approach")

    if not args.allow_descend:
        print("safe execution mode: moving to approach only; add --allow-descend for grasp/lift")
        execute_sequence(move_proxy, solved)
        return

    if args.use_gripper:
        maybe_set_gripper(float(get_param("grasp/open_gripper_pos_m", 0.065)), "open")
    execute_sequence(move_proxy, solved[:2])
    if args.use_gripper:
        maybe_set_gripper(float(get_param("grasp/close_gripper_pos_m", 0.028)), "close")
    execute_sequence(move_proxy, solved[2:])


def main():
    parser = argparse.ArgumentParser(description="Plan or execute a color-block grasp")
    parser.add_argument("command", choices=["plan", "execute"])
    parser.add_argument("--color", choices=["red", "green"], default=None)
    parser.add_argument("--execute", action="store_true", help="execute motion for command=plan")
    parser.add_argument("--allow-descend", action="store_true", help="allow descent from approach to grasp and lift")
    parser.add_argument("--use-gripper", action="store_true", help="open/close gripper during execution")
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.command == "execute":
        args.execute = True

    rospy.init_node("carm_a3_block_grasp")
    try:
        plan_block_grasp(args)
        return 0
    except (RuntimeError, ValueError, rospy.ROSException, rospy.ServiceException) as exc:
        print(exc, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

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
from carm_a3_motion.srv import MoveJointTrajectory
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


def slerp_quat_xyzw(start, end, ratio):
    return normalize_quat_xyzw([
        float(value) for value in tf.transformations.quaternion_slerp(
            normalize_quat_xyzw(start),
            normalize_quat_xyzw(end),
            float(ratio),
        )
    ])


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


def normalize_bidirectional_axis_delta(delta):
    while delta > math.pi * 0.5:
        delta -= math.pi
    while delta < -math.pi * 0.5:
        delta += math.pi
    return delta


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
    block_size = get_param("block/size_m", [0.10, 0.05, 0.05])
    block_height = block_height_from_config(block_size)
    projection_z = table_z + block_height
    point = project_pixel_to_plane(listener, camera_model, payload, detection["center_px"], projection_z)
    return [float(point[0]), float(point[1]), float(point[2])]


def estimate_block_axes(listener, detection, camera_model, payload):
    corners = detection.get("corners_px", [])
    if len(corners) != 4:
        return None
    table_z = float(get_param("workspace/table_z_m", 0.0))
    block_size = get_param("block/size_m", [0.10, 0.05, 0.05])
    projection_z = table_z + block_height_from_config(block_size)
    points = [
        project_pixel_to_plane(listener, camera_model, payload, corner, projection_z)
        for corner in corners
    ]
    edges = []
    for index in range(4):
        vec = points[(index + 1) % 4] - points[index]
        vec[2] = 0.0
        length = float(np.linalg.norm(vec))
        if length > 1e-9:
            unit = vec / length
            if unit[0] < 0.0:
                unit = -unit
            edges.append((length, unit))
    if len(edges) < 2:
        return None
    long_len, long_vec = max(edges, key=lambda item: item[0])
    short_len, short_vec = min(edges, key=lambda item: item[0])
    return {
        "long": {
            "direction": long_vec,
            "length": long_len,
            "yaw_deg": math.degrees(math.atan2(long_vec[1], long_vec[0])),
        },
        "short": {
            "direction": short_vec,
            "length": short_len,
            "yaw_deg": math.degrees(math.atan2(short_vec[1], short_vec[0])),
        },
    }


def block_height_from_config(block_size):
    if len(block_size) != 3:
        raise ValueError("block/size_m must contain long side, short side, height")
    return float(block_size[2])


def auto_tcp_grasp_z():
    mode = str(get_param("grasp/tcp_grasp_z_mode", "auto")).lower()
    if mode in ["manual", "fixed"]:
        value = float(get_param("grasp/tcp_grasp_z_m", 0.12))
        print("tcp grasp z manual: {:.6g}".format(value))
        return value
    table_z = float(get_param("workspace/table_z_m", 0.0))
    block_size = get_param("block/size_m", [0.10, 0.05, 0.05])
    block_height = block_height_from_config(block_size)
    stage = str(get_param("grasp/tcp_grasp_stage", "top_safe")).lower()
    if stage == "center":
        clearance = float(get_param("grasp/tcp_center_clearance_m", 0.0))
        value = table_z + block_height * 0.5 + clearance
        print(
            "tcp grasp z auto: stage=center, table_z={:.6g}, block_height={:.6g}, center_clearance={:.6g}, tcp_grasp_z={:.6g}".format(
                table_z,
                block_height,
                clearance,
                value,
            )
        )
        return value
    if stage in ["top", "top_safe"]:
        clearance = float(get_param("grasp/tcp_top_clearance_m", 0.015))
        value = table_z + block_height + clearance
        print(
            "tcp grasp z auto: stage={}, table_z={:.6g}, block_height={:.6g}, top_clearance={:.6g}, tcp_grasp_z={:.6g}".format(
                stage,
                table_z,
                block_height,
                clearance,
                value,
            )
        )
        return value
    raise ValueError("grasp/tcp_grasp_stage must be 'top_safe', 'top', or 'center'")


def select_grasp_axis(block_axes):
    if block_axes is None:
        return None, "none"
    if not bool(get_param("grasp/auto_select_grasp_side", True)):
        return block_axes["long"]["direction"], "manual_long_axis"
    block_size = get_param("block/size_m", [0.10, 0.05, 0.05])
    sorted_sides = sorted([float(value) for value in block_size])
    short_side = sorted_sides[0]
    long_side = sorted_sides[-1]
    max_open = float(get_param("grasp/max_gripper_open_m", 0.08))
    clearance = float(get_param("grasp/grasp_clearance_m", 0.005))
    if long_side + clearance > max_open and short_side + clearance <= max_open:
        return block_axes["short"]["direction"], "span_short_side"
    return block_axes["long"]["direction"], "span_long_side"


def align_quat_to_block_axis(quat, block_axes):
    align_axis = bool(get_param(
        "grasp/align_to_block_axis",
        get_param("grasp/align_to_block_long_edge", True),
    ))
    if block_axes is None or not align_axis:
        return quat
    target_axis, mode = select_grasp_axis(block_axes)
    if target_axis is None:
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
    delta = signed_angle_xy(current_axis, target_axis)
    if bool(get_param("grasp/treat_gripper_axis_bidirectional", True)):
        delta = normalize_bidirectional_axis_delta(delta)
    delta += math.radians(float(get_param("grasp/align_yaw_offset_deg", 0.0)))
    aligned = yaw_rotation(delta).dot(rotation)
    print(
        "orientation alignment: mode={}, tool_span_axis={}, target_axis={}, yaw_delta_deg={:.6g}".format(
            mode, axis_name, vector_to_text(target_axis), math.degrees(delta)
        )
    )
    return matrix_to_quat_xyzw(aligned)


def pose_for_tcp_target(tcp_xyz, quat, flange_to_tcp):
    rotation = quat_to_matrix(quat)
    offset = rotation.dot(np.asarray(flange_to_tcp, dtype=float))
    flange_xyz = np.asarray(tcp_xyz, dtype=float) - offset
    return [float(value) for value in flange_xyz] + quat


def constrain_top_safe_tcp_z(tcp_grasp_z, quat, flange_to_tcp):
    stage = str(get_param("grasp/tcp_grasp_stage", "top_safe")).lower()
    if stage not in ["top", "top_safe"]:
        return tcp_grasp_z
    min_flange_z = float(get_param("grasp/min_flange_z_m", 0.18))
    offset_z = float(quat_to_matrix(quat).dot(np.asarray(flange_to_tcp, dtype=float))[2])
    min_tcp_z = min_flange_z + offset_z
    if tcp_grasp_z >= min_tcp_z:
        return tcp_grasp_z
    print(
        "tcp grasp z raised by flange guard: requested={:.6g}, required={:.6g}, min_flange_z={:.6g}, tcp_offset_z_in_base={:.6g}".format(
            tcp_grasp_z,
            min_tcp_z,
            min_flange_z,
            offset_z,
        )
    )
    return min_tcp_z


def resolve_flange_to_tcp(listener):
    source = str(get_param("grasp/flange_to_tcp_source", "tf")).lower()
    tcp_frame = get_param("grasp/tcp_frame_id", "gripper_tcp")
    if source == "tf":
        try:
            listener.waitForTransform("flange", tcp_frame, rospy.Time(0), rospy.Duration(DEFAULT_TIMEOUT_S))
            translation, _ = listener.lookupTransform("flange", tcp_frame, rospy.Time(0))
            values = [float(value) for value in translation]
            print("flange_to_tcp from TF flange -> {}: {}".format(tcp_frame, vector_to_text(values)))
            return values
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            raise RuntimeError(
                "flange_to_tcp TF lookup failed for flange -> {}; restart robot_state_publisher/pregrasp_overview or set grasp/flange_to_tcp_source=config explicitly: {}".format(
                    tcp_frame,
                    exc,
                )
            )
    if source != "config":
        raise ValueError("grasp/flange_to_tcp_source must be 'tf' or 'config'")
    values = get_param("grasp/flange_to_tcp_xyz_m", [0.0, 0.0, 0.0])
    if len(values) != 3:
        raise ValueError("grasp/flange_to_tcp_xyz_m must contain 3 values")
    values = [float(value) for value in values]
    print("flange_to_tcp from config: {}".format(vector_to_text(values)))
    return values


def build_grasp_poses(point, current_pose, block_axes, allow_descend, flange_to_tcp):
    use_current_orientation = bool(get_param("grasp/use_current_orientation", True))
    if use_current_orientation:
        quat = list(current_pose[3:7])
    else:
        quat = get_param("grasp/target_quat_xyzw", [0.707, 0.0, 0.707, 0.0])
    quat = normalize_quat_xyzw([float(value) for value in quat])
    align_for_this_plan = allow_descend or not bool(get_param("grasp/align_only_when_descending", True))
    if align_for_this_plan:
        quat = align_quat_to_block_axis(quat, block_axes)
    elif block_axes is not None:
        print("approach-only mode: keeping current orientation; add --allow-descend to align gripper axis")

    approach_height = float(get_param("grasp/approach_height_m", 0.08))
    grasp_z = float(get_param("grasp/grasp_z_m", 0.22))
    lift_height = float(get_param("grasp/lift_height_m", 0.10))
    use_tcp_target = bool(get_param("grasp/use_tcp_target", False))

    x, y, _ = point
    if use_tcp_target:
        tcp_grasp_z = auto_tcp_grasp_z()
        tcp_grasp_z = constrain_top_safe_tcp_z(tcp_grasp_z, quat, flange_to_tcp)
        print(
            "tcp target enabled: flange_to_tcp={}, tcp_grasp_z={:.6g}".format(
                vector_to_text(flange_to_tcp), tcp_grasp_z
            )
        )
        grasp = pose_for_tcp_target([x, y, tcp_grasp_z], quat, flange_to_tcp)
        approach = pose_for_tcp_target([x, y, tcp_grasp_z + approach_height], quat, flange_to_tcp)
        lift = pose_for_tcp_target([x, y, tcp_grasp_z + lift_height], quat, flange_to_tcp)
    else:
        grasp = [x, y, grasp_z] + quat
        approach = [x, y, grasp_z + approach_height] + quat
        lift = [x, y, grasp_z + lift_height] + quat
    return {
        "approach": approach,
        "grasp": grasp,
        "lift": lift,
    }


def build_ordered_pose_sequence(poses, current_pose, allow_descend):
    ordered = []
    if allow_descend and bool(get_param("grasp/recenter_before_descent", True)):
        count = max(0, int(get_param("grasp/align_transit_waypoints", 6)))
        current_xyz = [float(value) for value in current_pose[:3]]
        current_quat = normalize_quat_xyzw([float(value) for value in current_pose[3:7]])
        approach_xyz = poses["approach"][:3]
        approach_quat = normalize_quat_xyzw(poses["approach"][3:7])
        for index in range(1, count + 1):
            ratio = float(index) / float(count + 1)
            xyz = [
                current_xyz[i] + (approach_xyz[i] - current_xyz[i]) * ratio
                for i in range(3)
            ]
            quat = slerp_quat_xyzw(current_quat, approach_quat, ratio)
            ordered.append(("align_transit_{}".format(index), xyz + quat))
    elif bool(get_param("grasp/keep_camera_view_during_approach", True)):
        count = int(get_param("grasp/view_transit_waypoints", 4))
        count = max(0, count)
        current_xyz = [float(value) for value in current_pose[:3]]
        current_quat = normalize_quat_xyzw([float(value) for value in current_pose[3:7]])
        approach_xyz = poses["approach"][:3]
        for index in range(1, count + 1):
            ratio = float(index) / float(count + 1)
            xyz = [
                current_xyz[i] + (approach_xyz[i] - current_xyz[i]) * ratio
                for i in range(3)
            ]
            ordered.append(("view_transit_{}".format(index), xyz + current_quat))

    ordered.append(("approach", poses["approach"]))
    if allow_descend:
        ordered.append(("grasp", poses["grasp"]))
        ordered.append(("lift", poses["lift"]))
    return ordered


def validate_pose_heights(ordered_poses):
    min_z = float(get_param("grasp/min_flange_z_m", 0.18))
    for name, pose in ordered_poses:
        if float(pose[2]) < min_z:
            raise RuntimeError(
                "{} flange z {:.6g} is below min_flange_z_m {:.6g}; refusing to plan near table".format(
                    name, float(pose[2]), min_z
                )
            )


def solve_pose_sequence(ik_proxy, ordered_poses, seed):
    tool_index = int(get_param("grasp/tool_index", 0))
    solved = []
    current_seed = list(seed)
    for name, pose in ordered_poses:
        res = ik_proxy(tool_index, pose, current_seed)
        print("{} ik result:".format(name))
        print(res)
        if not res.success:
            raise RuntimeError("{} IK failed: {}".format(name, res.message))
        positions = list(res.positions)
        solved.append((name, pose, positions, max_abs_delta(current_seed, positions)))
        current_seed = positions
    return solved


def validate_solved_sequence(solved, seed):
    max_total = float(get_param("grasp/max_total_joint_delta_rad", 2.20))
    max_segment_limit = float(get_param("grasp/max_segment_joint_delta_rad", max_total))
    first_delta = max_abs_delta(list(seed), solved[0][2])
    max_segment = max([first_delta] + [item[3] for item in solved[1:]])
    print("max_sequence_joint_delta: {:.9g}".format(max_segment))
    if max_segment > max_total:
        raise RuntimeError(
            "planned joint delta {:.9g} exceeds limit {:.9g}".format(max_segment, max_total)
        )
    if max_segment > max_segment_limit:
        raise RuntimeError(
            "planned segment joint delta {:.9g} exceeds segment limit {:.9g}; likely IK branch jump".format(
                max_segment, max_segment_limit
            )
        )


def print_joint_targets(solved):
    print("joint targets:")
    for name, _, joints, delta in solved:
        print("{} delta={:.9g}: {}".format(name, delta, vector_to_text(joints)))


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


def execute_trajectory(move_traj_proxy, solved):
    if not hasattr(execute_sequence, "last_positions"):
        raise RuntimeError("execute_sequence.last_positions is not set")
    segment_delta = float(get_param("grasp/segment_delta_rad", 0.08))
    flat = []
    expanded = []
    current = list(execute_sequence.last_positions)
    for name, _, target, _ in solved:
        waypoints = interpolate_joints(current, target, segment_delta)
        for waypoint in waypoints:
            expanded.append((name, waypoint))
            flat.extend(waypoint)
        current = target
    print("executing continuous joint trajectory: points={}".format(len(expanded)))
    res = move_traj_proxy(flat, len(expanded), [], False)
    print(res)
    if not res.success:
        raise RuntimeError("move_joint_trajectory failed: {}".format(res.message))
    execute_sequence.last_positions = list(expanded[-1][1])


def execute_motion(solved):
    prefer_trajectory = bool(get_param("grasp/prefer_joint_trajectory", True))
    if prefer_trajectory:
        try:
            move_traj_proxy = service_proxy(
                "/carm_a3/motion/move_joint_trajectory",
                MoveJointTrajectory,
            )
            execute_trajectory(move_traj_proxy, solved)
            return
        except (RuntimeError, rospy.ServiceException) as exc:
            if not bool(get_param("grasp/fallback_after_trajectory_failure", False)):
                raise
            print("trajectory execution unavailable, falling back to segmented move_joint: {}".format(exc))
    move_proxy = service_proxy("/carm_a3/motion/move_joint", MoveJoint)
    execute_sequence(move_proxy, solved)


def lookup_point(listener, target_frame, source_frame):
    try:
        listener.waitForTransform(target_frame, source_frame, rospy.Time(0), rospy.Duration(DEFAULT_TIMEOUT_S))
        translation, _ = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
        raise RuntimeError("TF {} -> {} unavailable: {}".format(target_frame, source_frame, exc))
    return [float(value) for value in translation]


def clamp_xy_step(error_xy, max_step):
    norm = math.sqrt(error_xy[0] * error_xy[0] + error_xy[1] * error_xy[1])
    if norm <= max_step or norm <= 1e-9:
        return error_xy, norm
    scale = max_step / norm
    return [error_xy[0] * scale, error_xy[1] * scale], norm


def visual_recenter_after_approach(
    listener,
    camera_model,
    detection_topic,
    target_color,
    min_confidence,
    cart_proxy,
    joint_proxy,
    ik_proxy,
):
    if not bool(get_param("grasp/visual_recenter_after_approach", True)):
        return

    iterations = max(1, int(get_param("grasp/recenter_iterations", 1)))
    base_frame = get_param("workspace/base_frame_id", "base_link")
    tcp_frame = get_param("grasp/tcp_frame_id", "gripper_tcp")
    for iteration in range(1, iterations + 1):
        payload = wait_json(detection_topic)
        detection = choose_detection(payload, target_color, min_confidence)
        block_point = project_detection_to_table(listener, detection, camera_model, payload)
        tcp_point = lookup_point(listener, base_frame, tcp_frame)
        error_xy = [
            float(block_point[0]) - float(tcp_point[0]),
            float(block_point[1]) - float(tcp_point[1]),
        ]
        max_step = float(get_param("grasp/recenter_max_step_m", 0.04))
        tolerance = float(get_param("grasp/recenter_tolerance_m", 0.006))
        step_xy, raw_norm = clamp_xy_step(error_xy, max_step)
        step_norm = math.sqrt(step_xy[0] * step_xy[0] + step_xy[1] * step_xy[1])
        print("visual recenter iteration {}/{}:".format(iteration, iterations))
        print("block_xy={}, tcp_xy={}, error_xy={}, error_norm={:.6g}".format(
            vector_to_text(block_point[:2]),
            vector_to_text(tcp_point[:2]),
            vector_to_text(error_xy),
            raw_norm,
        ))
        if raw_norm <= tolerance:
            print("visual recenter done: error within tolerance {:.6g} m".format(tolerance))
            return
        if step_norm <= 1e-9:
            print("visual recenter stopped: near-zero clamped step")
            return

        cart_res = cart_proxy()
        if not cart_res.success:
            raise RuntimeError("visual recenter cartesian snapshot failed: {}".format(cart_res.message))
        joint_res = joint_proxy()
        if not joint_res.success:
            raise RuntimeError("visual recenter joint snapshot failed: {}".format(joint_res.message))
        target_pose = list(cart_res.pose)
        target_pose[0] += step_xy[0]
        target_pose[1] += step_xy[1]
        tool_index = int(get_param("grasp/tool_index", 0))
        ik_res = ik_proxy(tool_index, target_pose, list(joint_res.positions))
        print("visual recenter ik result:")
        print(ik_res)
        if not ik_res.success:
            raise RuntimeError("visual recenter IK failed: {}".format(ik_res.message))
        move_proxy = service_proxy("/carm_a3/motion/move_joint", MoveJoint)
        duration = float(get_param("grasp/recenter_duration_s", 1.5))
        print("visual recenter executing step_xy={}, clamped_from={:.6g}".format(
            vector_to_text(step_xy),
            raw_norm,
        ))
        res = move_proxy(list(ik_res.positions), duration, False)
        print(res)
        if not res.success:
            raise RuntimeError("visual recenter move failed: {}".format(res.message))
        rospy.sleep(0.2)

    print("visual recenter reached iteration limit; rerun approach-only if more correction is needed")


def observe_block(listener, camera_model, detection_topic, target_color, min_confidence, label):
    payload = wait_json(detection_topic)
    detection = choose_detection(payload, target_color, min_confidence)
    print("{} selected detection:".format(label))
    print(json.dumps(detection, sort_keys=True))
    point = project_detection_to_table(listener, detection, camera_model, payload)
    print("{} estimated block top center in base frame x,y,z:".format(label))
    print(vector_to_text(point))
    block_axes = estimate_block_axes(listener, detection, camera_model, payload)
    if block_axes is not None:
        print("{} estimated block axes in base frame:".format(label))
        print("long: direction={}, length_m={:.9g}, yaw_deg={:.6g}".format(
            vector_to_text(block_axes["long"]["direction"]),
            block_axes["long"]["length"],
            block_axes["long"]["yaw_deg"],
        ))
        print("short: direction={}, length_m={:.9g}, yaw_deg={:.6g}".format(
            vector_to_text(block_axes["short"]["direction"]),
            block_axes["short"]["length"],
            block_axes["short"]["yaw_deg"],
        ))
        selected_axis, selected_mode = select_grasp_axis(block_axes)
        if selected_axis is not None:
            print("{} selected gripper span axis: mode={}, direction={}".format(
                label,
                selected_mode,
                vector_to_text(selected_axis),
            ))
    return payload, detection, point, block_axes


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

    camera_model = wait_camera_info(camera_info_topic)

    listener = tf.TransformListener()
    rospy.sleep(0.2)
    _, _, point, block_axes = observe_block(
        listener,
        camera_model,
        detection_topic,
        target_color,
        min_confidence,
        "initial",
    )

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

    two_stage_descent = (
        args.execute
        and args.allow_descend
        and bool(get_param("grasp/recenter_before_descent", True))
    )
    planning_allow_descend = args.allow_descend and not two_stage_descent
    if two_stage_descent:
        print("two-stage execution enabled: initial plan is approach-only; descent is replanned after visual recenter")

    flange_to_tcp = resolve_flange_to_tcp(listener)
    poses = build_grasp_poses(
        point,
        list(cart_res.pose),
        block_axes,
        planning_allow_descend,
        flange_to_tcp,
    )
    print("planned poses x,y,z,qx,qy,qz,qw:")
    for name in ["approach", "grasp", "lift"]:
        print("{}: {}".format(name, vector_to_text(poses[name])))

    ordered_poses = build_ordered_pose_sequence(poses, list(cart_res.pose), planning_allow_descend)
    if two_stage_descent:
        print("safe first stage: planning visual approach only; aligned descent will be replanned after recenter")
    elif not planning_allow_descend:
        print("safe planning mode: planning approach only; add --allow-descend to include grasp/lift")
    print("ordered execution poses:")
    for name, pose in ordered_poses:
        print("{}: {}".format(name, vector_to_text(pose)))
    validate_pose_heights(ordered_poses)

    solved = solve_pose_sequence(ik_proxy, ordered_poses, list(joint_res.positions))
    validate_solved_sequence(solved, list(joint_res.positions))
    print_joint_targets(solved)

    if not args.execute:
        print("not executing; rerun with --execute after checking target and clearance")
        return

    execute_sequence.last_positions = list(joint_res.positions)
    if bool(get_param("grasp/open_before_approach", True)):
        maybe_set_gripper(float(get_param("grasp/open_gripper_pos_m", 0.065)), "open-before-approach")

    if not args.allow_descend:
        print("safe execution mode: moving to approach only; add --allow-descend for grasp/lift")
        execute_motion(solved)
        visual_recenter_after_approach(
            listener,
            camera_model,
            detection_topic,
            target_color,
            min_confidence,
            cart_proxy,
            joint_proxy,
            ik_proxy,
        )
        return

    if two_stage_descent:
        print("two-stage execution: first moving to visual approach, then recentering before descent")
        execute_motion(solved)
        visual_recenter_after_approach(
            listener,
            camera_model,
            detection_topic,
            target_color,
            min_confidence,
            cart_proxy,
            joint_proxy,
            ik_proxy,
        )
        _, _, point, block_axes = observe_block(
            listener,
            camera_model,
            detection_topic,
            target_color,
            min_confidence,
            "post-recenter",
        )
        cart_res = cart_proxy()
        print("post-recenter cartesian snapshot:")
        print(cart_res)
        if not cart_res.success:
            raise RuntimeError(cart_res.message)
        joint_res = joint_proxy()
        print("post-recenter joint snapshot:")
        print(joint_res)
        if not joint_res.success:
            raise RuntimeError(joint_res.message)
        flange_to_tcp = resolve_flange_to_tcp(listener)
        poses = build_grasp_poses(point, list(cart_res.pose), block_axes, True, flange_to_tcp)
        print("post-recenter planned poses x,y,z,qx,qy,qz,qw:")
        for name in ["approach", "grasp", "lift"]:
            print("{}: {}".format(name, vector_to_text(poses[name])))
        ordered_poses = build_ordered_pose_sequence(poses, list(cart_res.pose), True)
        print("post-recenter ordered execution poses:")
        for name, pose in ordered_poses:
            print("{}: {}".format(name, vector_to_text(pose)))
        validate_pose_heights(ordered_poses)
        solved = solve_pose_sequence(ik_proxy, ordered_poses, list(joint_res.positions))
        validate_solved_sequence(solved, list(joint_res.positions))
        print_joint_targets(solved)
        execute_sequence.last_positions = list(joint_res.positions)

    if args.use_gripper:
        maybe_set_gripper(float(get_param("grasp/open_gripper_pos_m", 0.065)), "open")
    grasp_index = next(
        (index for index, item in enumerate(solved) if item[0] == "grasp"),
        None,
    )
    if grasp_index is None:
        raise RuntimeError("internal error: grasp target missing from solved sequence")
    execute_motion(solved[:grasp_index + 1])
    if args.use_gripper:
        maybe_set_gripper(float(get_param("grasp/close_gripper_pos_m", 0.028)), "close")
    execute_motion(solved[grasp_index + 1:])


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

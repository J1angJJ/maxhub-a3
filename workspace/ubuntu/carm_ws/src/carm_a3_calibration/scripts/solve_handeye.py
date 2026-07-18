#!/usr/bin/env python3
import argparse
import glob
import math
import os
from datetime import datetime

import numpy as np
import yaml

try:
    import cv2
except ImportError as exc:
    raise SystemExit("python3-opencv is required: sudo apt install python3-opencv") from exc


METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def read_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError("{} is not a YAML mapping".format(path))
    return data


def vector_from_dict(data):
    return np.array([data["x"], data["y"], data["z"]], dtype=np.float64).reshape(3, 1)


def rotation_from_quat(q):
    x, y, z, w = float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("zero quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def quat_from_rotation(rot):
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    quat = np.array([x, y, z, w], dtype=np.float64)
    quat /= np.linalg.norm(quat)
    return quat


def angle_from_rotation(rot):
    value = (np.trace(rot) - 1.0) / 2.0
    value = max(-1.0, min(1.0, float(value)))
    return math.degrees(math.acos(value))


def load_sample(path):
    data = read_yaml(path)
    base_t_flange = data["base_T_flange"]
    camera_t_marker = data["camera_T_marker"]
    return {
        "path": path,
        "base_R_flange": rotation_from_quat(base_t_flange["rotation"]),
        "base_t_flange": vector_from_dict(base_t_flange["translation"]),
        "camera_R_marker": np.asarray(camera_t_marker["rotation_matrix"], dtype=np.float64),
        "camera_t_marker": np.asarray(camera_t_marker["tvec"], dtype=np.float64).reshape(3, 1),
    }


def describe_motion(samples):
    flange_positions = np.hstack([s["base_t_flange"] for s in samples]).T
    marker_positions = np.hstack([s["camera_t_marker"] for s in samples]).T
    flange_angles = [angle_from_rotation(samples[0]["base_R_flange"].T @ s["base_R_flange"]) for s in samples[1:]]
    return {
        "flange_translation_span_m": (flange_positions.max(axis=0) - flange_positions.min(axis=0)).tolist(),
        "marker_depth_range_m": [float(marker_positions[:, 2].min()), float(marker_positions[:, 2].max())],
        "max_relative_flange_rotation_deg": float(max(flange_angles)) if flange_angles else 0.0,
    }


def make_matrix(rot, trans):
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rot
    out[:3, 3] = trans.reshape(3)
    return out


def solve(samples, method):
    return cv2.calibrateHandEye(
        [s["base_R_flange"] for s in samples],
        [s["base_t_flange"] for s in samples],
        [s["camera_R_marker"] for s in samples],
        [s["camera_t_marker"] for s in samples],
        method=method,
    )


def result_dict(method_name, rot, trans):
    quat = quat_from_rotation(rot)
    return {
        "method": method_name,
        "parent_frame": "flange",
        "child_frame": "carm_a3_camera_optical_frame",
        "translation": {
            "x": float(trans[0]),
            "y": float(trans[1]),
            "z": float(trans[2]),
        },
        "rotation": {
            "x": float(quat[0]),
            "y": float(quat[1]),
            "z": float(quat[2]),
            "w": float(quat[3]),
        },
        "matrix": make_matrix(rot, trans).tolist(),
    }


def translation_from_result(result):
    trans = result["translation"]
    return np.array([trans["x"], trans["y"], trans["z"]], dtype=np.float64)


def rotation_from_result(result):
    return rotation_from_quat(result["rotation"])


def compare_to_reference(method_results, reference):
    comparisons = []
    ref_rot = rotation_from_result(reference)
    ref_trans = translation_from_result(reference)
    for result in method_results:
        if "error" in result:
            continue
        rot = rotation_from_result(result)
        trans = translation_from_result(result)
        comparisons.append(
            {
                "method": result["method"],
                "translation_delta_m": float(np.linalg.norm(trans - ref_trans)),
                "rotation_delta_deg": float(angle_from_rotation(ref_rot.T @ rot)),
            }
        )
    return comparisons


def parse_args():
    parser = argparse.ArgumentParser(description="Solve eye-in-hand calibration from CArm A3 ArUco YAML samples.")
    parser.add_argument(
        "--samples-dir",
        default=os.path.expanduser("~/maxhub-a3/workspace/ubuntu/logs/handeye_samples"),
        help="Directory containing sample_*.yaml files.",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output YAML path. Defaults to <samples-dir>/handeye_result.yaml.",
    )
    parser.add_argument(
        "--method",
        choices=sorted(METHODS.keys()),
        default="PARK",
        help="Primary OpenCV hand-eye method for the top-level result.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    sample_paths = sorted(glob.glob(os.path.join(os.path.expanduser(args.samples_dir), "sample_*.yaml")))
    if len(sample_paths) < 3:
        raise SystemExit("Need at least 3 samples, found {}".format(len(sample_paths)))

    samples = [load_sample(path) for path in sample_paths]
    method_results = []
    for name, method in METHODS.items():
        try:
            rot, trans = solve(samples, method)
            method_results.append(result_dict(name, rot, trans.reshape(3)))
        except cv2.error as exc:
            method_results.append({"method": name, "error": str(exc)})

    primary = next(item for item in method_results if item["method"] == args.method and "error" not in item)
    output = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "sample_count": len(samples),
        "samples_dir": os.path.abspath(os.path.expanduser(args.samples_dir)),
        "input_convention": {
            "robot_pose": "base_T_flange",
            "target_pose": "camera_T_marker",
        },
        "recommended_transform": primary,
        "all_methods": method_results,
        "method_consistency_to_recommended": compare_to_reference(method_results, primary),
        "motion_summary": describe_motion(samples),
    }

    output_path = args.output or os.path.join(os.path.expanduser(args.samples_dir), "handeye_result.yaml")
    with open(output_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(output, f, sort_keys=False, allow_unicode=True)

    print("samples:", len(samples))
    print("primary method:", args.method)
    print("flange -> carm_a3_camera_optical_frame")
    print("translation m:", primary["translation"])
    print("rotation xyzw:", primary["rotation"])
    print("written:", output_path)


if __name__ == "__main__":
    main()

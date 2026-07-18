#!/usr/bin/env python3
import os
import sys
import threading
from datetime import datetime

import numpy as np
import rospy
import tf2_ros
import yaml
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger, TriggerResponse

try:
    import cv2
except ImportError as exc:
    raise SystemExit("python3-opencv is required: sudo apt install python3-opencv") from exc


def get_aruco_dictionary():
    if not hasattr(cv2, "aruco"):
        raise SystemExit("cv2.aruco is missing. Install OpenCV contrib modules, e.g. sudo apt install python3-opencv")
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)


def get_detector_parameters():
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        return cv2.aruco.DetectorParameters_create()
    return cv2.aruco.DetectorParameters()


def image_to_rgb(msg):
    if msg.encoding == "rgb8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if msg.encoding == "mono8":
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    raise ValueError("unsupported image encoding: {}".format(msg.encoding))


def transform_to_dict(transform):
    return {
        "parent_frame": transform.header.frame_id,
        "child_frame": transform.child_frame_id,
        "stamp": {
            "secs": int(transform.header.stamp.secs),
            "nsecs": int(transform.header.stamp.nsecs),
        },
        "translation": {
            "x": float(transform.transform.translation.x),
            "y": float(transform.transform.translation.y),
            "z": float(transform.transform.translation.z),
        },
        "rotation": {
            "x": float(transform.transform.rotation.x),
            "y": float(transform.transform.rotation.y),
            "z": float(transform.transform.rotation.z),
            "w": float(transform.transform.rotation.w),
        },
    }


def marker_pose_to_dict(camera_frame, stamp, rvec, tvec):
    rot_mat, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    return {
        "parent_frame": camera_frame,
        "child_frame": "aruco_marker",
        "stamp": {
            "secs": int(stamp.secs),
            "nsecs": int(stamp.nsecs),
        },
        "translation": {
            "x": float(tvec[0]),
            "y": float(tvec[1]),
            "z": float(tvec[2]),
        },
        "rotation_matrix": [
            [float(rot_mat[0, 0]), float(rot_mat[0, 1]), float(rot_mat[0, 2])],
            [float(rot_mat[1, 0]), float(rot_mat[1, 1]), float(rot_mat[1, 2])],
            [float(rot_mat[2, 0]), float(rot_mat[2, 1]), float(rot_mat[2, 2])],
        ],
        "rvec": [float(rvec[0]), float(rvec[1]), float(rvec[2])],
        "tvec": [float(tvec[0]), float(tvec[1]), float(tvec[2])],
    }


class ArucoHandeyeSampler:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/carm_a3/camera/image_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/carm_a3/camera/camera_info")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.flange_frame = rospy.get_param("~flange_frame", "flange")
        self.camera_frame = rospy.get_param("~camera_frame", "carm_a3_camera_optical_frame")
        self.marker_id = int(rospy.get_param("~marker_id", 23))
        self.marker_size_m = float(rospy.get_param("~marker_size_m", 0.1))
        self.output_dir = os.path.expanduser(rospy.get_param("~output_dir", "~/maxhub-a3/workspace/ubuntu/logs/handeye_samples"))
        self.save_sample_service = rospy.get_param("~save_sample_service", "/carm_a3/handeye/save_sample")
        self.max_sample_age_s = float(rospy.get_param("~max_sample_age_s", 1.0))

        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_sample = None
        self.sample_count = 0
        self.lock = threading.Lock()

        self.dictionary = get_aruco_dictionary()
        self.detector_parameters = get_detector_parameters()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        os.makedirs(self.output_dir, exist_ok=True)
        self.sample_count = self.count_existing_samples()
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        rospy.Service(self.save_sample_service, Trigger, self.save_sample_cb)

        rospy.loginfo("Aruco hand-eye sampler waiting for marker id=%d size=%.3fm", self.marker_id, self.marker_size_m)
        rospy.loginfo("Save samples with: rosservice call %s", self.save_sample_service)
        rospy.loginfo("If running with rosrun, pressing Enter in this terminal also saves the latest valid sample.")
        threading.Thread(target=self.input_loop, daemon=True).start()

    def count_existing_samples(self):
        names = [name for name in os.listdir(self.output_dir) if name.startswith("sample_") and name.endswith(".yaml")]
        return len(names)

    def camera_info_cb(self, msg):
        if len(msg.K) == 9 and msg.K[0] != 0.0:
            self.camera_matrix = np.asarray(msg.K, dtype=np.float64).reshape(3, 3)
            self.dist_coeffs = np.asarray(msg.D, dtype=np.float64).reshape(-1, 1)

    def image_cb(self, msg):
        if self.camera_matrix is None:
            rospy.logwarn_throttle(5.0, "waiting for calibrated camera_info")
            return

        try:
            rgb = image_to_rgb(msg)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, str(exc))
            return

        gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.detector_parameters)
        if ids is None:
            rospy.loginfo_throttle(2.0, "waiting for ArUco marker id=%d", self.marker_id)
            return

        ids_flat = ids.flatten()
        matches = np.where(ids_flat == self.marker_id)[0]
        if len(matches) == 0:
            rospy.loginfo_throttle(2.0, "detected markers %s, waiting for id=%d", ids_flat.tolist(), self.marker_id)
            return

        index = int(matches[0])
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            [corners[index]], self.marker_size_m, self.camera_matrix, self.dist_coeffs
        )

        try:
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.flange_frame, rospy.Time(0), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(2.0, "waiting for TF %s -> %s: %s", self.base_frame, self.flange_frame, exc)
            return

        sample = {
            "sample_index": self.sample_count + 1,
            "captured_at": datetime.now().isoformat(timespec="seconds"),
            "image_stamp": {
                "secs": int(msg.header.stamp.secs),
                "nsecs": int(msg.header.stamp.nsecs),
            },
            "image_topic": self.image_topic,
            "camera_info_topic": self.camera_info_topic,
            "marker": {
                "dictionary": "DICT_ARUCO_ORIGINAL",
                "id": self.marker_id,
                "size_m": self.marker_size_m,
            },
            "base_T_flange": transform_to_dict(transform),
            "camera_T_marker": marker_pose_to_dict(self.camera_frame, msg.header.stamp, rvecs[0][0], tvecs[0][0]),
        }
        with self.lock:
            self.latest_sample = sample
        rospy.loginfo_throttle(
            1.0,
            "valid marker id=%d: t=[%.3f %.3f %.3f] m; call save_sample to save",
            self.marker_id,
            sample["camera_T_marker"]["translation"]["x"],
            sample["camera_T_marker"]["translation"]["y"],
            sample["camera_T_marker"]["translation"]["z"],
        )

    def input_loop(self):
        while not rospy.is_shutdown():
            line = sys.stdin.readline()
            if line == "":
                rospy.sleep(0.2)
                continue
            self.save_latest_sample()

    def save_sample_cb(self, _req):
        success, message = self.save_latest_sample()
        return TriggerResponse(success=success, message=message)

    def save_latest_sample(self):
        with self.lock:
            sample = self.latest_sample
        if sample is None:
            message = "no valid sample yet; keep the marker visible and try again"
            rospy.logwarn(message)
            return False, message

        image_stamp = rospy.Time(sample["image_stamp"]["secs"], sample["image_stamp"]["nsecs"])
        age = abs((rospy.Time.now() - image_stamp).to_sec())
        if age > self.max_sample_age_s:
            message = "latest sample is {:.2f}s old; keep the marker visible and try again".format(age)
            rospy.logwarn(message)
            return False, message

        self.sample_count += 1
        sample["sample_index"] = self.sample_count
        filename = "sample_{:03d}.yaml".format(self.sample_count)
        path = os.path.join(self.output_dir, filename)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(sample, f, sort_keys=False, allow_unicode=True)
        message = "saved sample {:03d}: {}".format(self.sample_count, path)
        rospy.loginfo(message)
        return True, message


def main():
    rospy.init_node("aruco_handeye_sampler")
    ArucoHandeyeSampler()
    rospy.spin()


if __name__ == "__main__":
    main()

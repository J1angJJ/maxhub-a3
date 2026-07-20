#!/usr/bin/env python3
import json
import math
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def param(name, default):
    return rospy.get_param("~" + name, default)


def load_ranges(name):
    values = param(name, [])
    ranges = []
    for item in values:
        if len(item) != 6:
            rospy.logwarn("Ignoring invalid HSV range for %s: %s", name, item)
            continue
        lower = np.array(item[:3], dtype=np.uint8)
        upper = np.array(item[3:], dtype=np.uint8)
        ranges.append((lower, upper))
    return ranges


def box_points(rect):
    points = cv2.boxPoints(rect)
    return [[float(x), float(y)] for x, y in points]


def ordered_box_points(points):
    pts = np.asarray(points, dtype=np.float64)
    center = np.mean(pts, axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    ordered = pts[np.argsort(angles)]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    ordered = np.roll(ordered, -start, axis=0)
    return [[float(x), float(y)] for x, y in ordered]


def eight_points_from_corners(corners):
    if len(corners) != 4:
        return []
    pts = [np.asarray(point, dtype=np.float64) for point in corners]
    out = []
    for index in range(4):
        p0 = pts[index]
        p1 = pts[(index + 1) % 4]
        mid = (p0 + p1) * 0.5
        out.append([float(p0[0]), float(p0[1])])
        out.append([float(mid[0]), float(mid[1])])
    return out


def rect_axes(corners):
    if len(corners) != 4:
        return None
    edges = []
    for index in range(4):
        p0 = np.array(corners[index], dtype=np.float64)
        p1 = np.array(corners[(index + 1) % 4], dtype=np.float64)
        vec = p1 - p0
        length = float(np.linalg.norm(vec))
        if length <= 1e-9:
            continue
        edges.append((length, [float(vec[0] / length), float(vec[1] / length)]))
    if len(edges) < 2:
        return None
    long_edge = max(edges, key=lambda item: item[0])
    short_edge = min(edges, key=lambda item: item[0])
    return {
        "long_edge_px": {
            "length": long_edge[0],
            "direction": long_edge[1],
        },
        "short_edge_px": {
            "length": short_edge[0],
            "direction": short_edge[1],
        },
    }


def robust_box_from_contour(contour, percentile):
    pts = contour.reshape(-1, 2).astype(np.float64)
    if pts.shape[0] < 8:
        return None

    center = np.mean(pts, axis=0)
    shifted = pts - center
    cov = np.cov(shifted.T)
    values, vectors = np.linalg.eigh(cov)
    order = np.argsort(values)[::-1]
    axes = vectors[:, order]
    long_axis = axes[:, 0]
    short_axis = axes[:, 1]
    if long_axis[0] < 0:
        long_axis = -long_axis
    cross_z = long_axis[0] * short_axis[1] - long_axis[1] * short_axis[0]
    if cross_z < 0:
        short_axis = -short_axis

    proj_long = shifted.dot(long_axis)
    proj_short = shifted.dot(short_axis)
    pct = max(0.0, min(20.0, float(percentile)))
    long_min, long_max = np.percentile(proj_long, [pct, 100.0 - pct])
    short_min, short_max = np.percentile(proj_short, [pct, 100.0 - pct])
    if long_max <= long_min or short_max <= short_min:
        return None

    corners = [
        center + long_min * long_axis + short_min * short_axis,
        center + long_max * long_axis + short_min * short_axis,
        center + long_max * long_axis + short_max * short_axis,
        center + long_min * long_axis + short_max * short_axis,
    ]
    corners = ordered_box_points(corners)
    long_len = float(long_max - long_min)
    short_len = float(short_max - short_min)
    if short_len > long_len:
        long_len, short_len = short_len, long_len
        long_axis, short_axis = short_axis, long_axis
    return {
        "corners_px": corners,
        "corners8_px": eight_points_from_corners(corners),
        "long_edge_px": {
            "length": long_len,
            "direction": [float(long_axis[0]), float(long_axis[1])],
        },
        "short_edge_px": {
            "length": short_len,
            "direction": [float(short_axis[0]), float(short_axis[1])],
        },
        "rect_size_px": [long_len, short_len],
    }


class ColorBlockSegmenter:
    def __init__(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.camera_info = None

        self.image_topic = param("image_topic", "/carm_a3/camera/image_raw")
        self.camera_info_topic = param("camera_info_topic", "/carm_a3/camera/camera_info")
        self.detections_topic = param("detections_topic", "/carm_a3/perception/color_blocks")
        self.debug_image_topic = param("debug_image_topic", "/carm_a3/perception/color_blocks/debug_image")
        self.diagnostics_topic = param("diagnostics_topic", "/carm_a3/perception/color_blocks/diagnostics")
        self.frame_id = param("frame_id", "carm_a3_camera_optical_frame")

        self.publish_debug_image = bool(param("publish_debug_image", True))
        self.min_area_px = float(param("min_area_px", 500.0))
        self.max_area_px = float(param("max_area_px", 200000.0))
        self.min_extent = float(param("min_extent", 0.35))
        self.min_rect_short_side_px = float(param("min_rect_short_side_px", 12.0))
        self.max_blocks_per_color = int(param("max_blocks_per_color", 5))
        self.morph_kernel_size = int(param("morph_kernel_size", 5))
        self.median_blur_size = int(param("median_blur_size", 3))
        self.robust_box_percentile = float(param("robust_box_percentile", 2.0))
        self.min_color_fill_ratio = float(param("min_color_fill_ratio", 0.45))

        self.color_ranges = {
            "red": load_ranges("red_ranges"),
            "green": load_ranges("green_ranges"),
        }

        self.det_pub = rospy.Publisher(self.detections_topic, String, queue_size=10)
        self.diag_pub = rospy.Publisher(self.diagnostics_topic, String, queue_size=10, latch=True)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)

        self.info_sub = rospy.Subscriber(
            self.camera_info_topic, CameraInfo, self.on_camera_info, queue_size=1
        )
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.on_image, queue_size=1)

        self.publish_diag(
            "color_block_segmenter_started,image_topic={},debug={}".format(
                self.image_topic, self.publish_debug_image
            )
        )

    def publish_diag(self, text):
        msg = String()
        msg.data = text
        self.diag_pub.publish(msg)

    def on_camera_info(self, msg):
        with self.lock:
            self.camera_info = msg

    def make_mask(self, hsv, ranges):
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        if self.median_blur_size > 1:
            k = self.median_blur_size
            if k % 2 == 0:
                k += 1
            mask = cv2.medianBlur(mask, k)
        if self.morph_kernel_size > 1:
            k = self.morph_kernel_size
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def contour_detection(self, color, contour):
        area = float(cv2.contourArea(contour))
        if area < self.min_area_px or area > self.max_area_px:
            return None

        rect = cv2.minAreaRect(contour)
        (cx, cy), (w, h), angle = rect
        if w <= 0.0 or h <= 0.0:
            return None
        if min(w, h) < self.min_rect_short_side_px:
            return None

        rect_area = float(w * h)
        extent = area / rect_area if rect_area > 0.0 else 0.0
        if extent < self.min_extent:
            return None

        robust = robust_box_from_contour(contour, self.robust_box_percentile)
        if robust is None:
            corners = box_points(rect)
            corners8 = eight_points_from_corners(corners)
            robust_axes = rect_axes(corners)
            robust_rect_size = [float(w), float(h)]
        else:
            corners = robust["corners_px"]
            corners8 = robust["corners8_px"]
            robust_axes = {
                "long_edge_px": robust["long_edge_px"],
                "short_edge_px": robust["short_edge_px"],
            }
            robust_rect_size = robust["rect_size_px"]

        robust_area = max(float(robust_rect_size[0] * robust_rect_size[1]), 1e-6)
        color_fill_ratio = max(0.0, min(1.0, area / robust_area))
        if color_fill_ratio < self.min_color_fill_ratio:
            return None

        robust_aspect = max(robust_rect_size) / max(min(robust_rect_size), 1e-6)
        confidence = (
            max(0.0, min(1.0, extent))
            * max(0.0, min(1.0, color_fill_ratio))
            * max(0.0, min(1.0, 2.5 / robust_aspect))
        )

        moments = cv2.moments(contour)
        if abs(moments["m00"]) > 1e-9:
            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]

        detection = {
            "color": color,
            "center_px": [float(cx), float(cy)],
            "corners_px": corners,
            "corners8_px": corners8,
            "area_px": area,
            "rect_size_px": [float(robust_rect_size[0]), float(robust_rect_size[1])],
            "min_area_rect_size_px": [float(w), float(h)],
            "angle_deg": float(angle),
            "extent": float(extent),
            "color_fill_ratio": float(color_fill_ratio),
            "aspect": float(robust_aspect),
            "confidence": float(confidence),
        }
        if robust_axes is not None:
            detection.update(robust_axes)
        return detection

    def detect(self, bgr):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        detections = []
        masks = {}

        for color, ranges in self.color_ranges.items():
            mask = self.make_mask(hsv, ranges)
            masks[color] = mask
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            color_detections = []
            for contour in contours:
                det = self.contour_detection(color, contour)
                if det is not None:
                    color_detections.append(det)
            color_detections.sort(key=lambda item: item["area_px"], reverse=True)
            detections.extend(color_detections[: self.max_blocks_per_color])

        detections.sort(key=lambda item: item["confidence"], reverse=True)
        return detections, masks

    def draw_debug(self, bgr, detections):
        debug = bgr.copy()
        palette = {
            "red": (0, 0, 255),
            "green": (0, 220, 0),
        }
        for det in detections:
            color = palette.get(det["color"], (255, 255, 255))
            corners = np.array(det["corners_px"], dtype=np.int32)
            cv2.polylines(debug, [corners], True, color, 2)
            center = tuple(int(round(v)) for v in det["center_px"])
            cv2.circle(debug, center, 4, color, -1)
            for index, corner in enumerate(corners):
                cv2.circle(debug, tuple(corner), 3, (255, 255, 255), -1)
                cv2.putText(debug, str(index), tuple(corner + np.array([4, -4])),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            corners8 = np.array(det.get("corners8_px", []), dtype=np.int32)
            for index, point in enumerate(corners8):
                cv2.circle(debug, tuple(point), 2, (255, 255, 0), -1)
                if index % 2 == 1:
                    cv2.putText(debug, str(index), tuple(point + np.array([3, 3])),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 0), 1, cv2.LINE_AA)
            for axis_key, axis_color, axis_label in [
                ("long_edge_px", (255, 0, 0), "L"),
                ("short_edge_px", (0, 255, 255), "S"),
            ]:
                axis = det.get(axis_key)
                if axis is None:
                    continue
                direction = axis.get("direction", [0.0, 0.0])
                length = float(axis.get("length", 0.0)) * 0.35
                end = (
                    int(round(center[0] + float(direction[0]) * length)),
                    int(round(center[1] + float(direction[1]) * length)),
                )
                cv2.arrowedLine(debug, center, end, axis_color, 2, tipLength=0.25)
                cv2.putText(debug, axis_label, (end[0] + 4, end[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, axis_color, 1, cv2.LINE_AA)
            label = "{} {:.2f}".format(det["color"], det["confidence"])
            cv2.putText(debug, label, (center[0] + 6, center[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        return debug

    def on_image(self, msg):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.publish_diag("cv_bridge_error={}".format(exc))
            return

        detections, _ = self.detect(bgr)
        stamp = msg.header.stamp if msg.header.stamp else rospy.Time.now()
        frame_id = msg.header.frame_id or self.frame_id

        payload = {
            "header": {
                "stamp": {"secs": stamp.secs, "nsecs": stamp.nsecs},
                "frame_id": frame_id,
            },
            "image_topic": self.image_topic,
            "block_size_m": param("block_size_m", {"x": 0.05, "y": 0.05, "z": 0.10}),
            "detections": detections,
        }
        out = String()
        out.data = json.dumps(payload, sort_keys=True)
        self.det_pub.publish(out)

        self.publish_diag("detections={},red_green_segmenter_ok".format(len(detections)))

        if self.debug_pub is not None:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(self.draw_debug(bgr, detections), encoding="bgr8")
                debug_msg.header = msg.header
                if not debug_msg.header.frame_id:
                    debug_msg.header.frame_id = frame_id
                self.debug_pub.publish(debug_msg)
            except CvBridgeError as exc:
                self.publish_diag("debug_cv_bridge_error={}".format(exc))


def main():
    rospy.init_node("carm_a3_color_block_segmenter")
    ColorBlockSegmenter()
    rospy.spin()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import importlib.util
import math
import os
import sys
import threading
import time

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from carm_a3_motion.srv import JogJoint, JogJointResponse
from carm_a3_motion.srv import MoveJoint, MoveJointResponse


def load_vendor_carm_class():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    package_dir = os.path.dirname(script_dir)
    ws_dir = os.path.abspath(os.path.join(package_dir, "..", ".."))
    carm_py = os.path.join(ws_dir, "vendor", "arm_control_sdk", "python", "carm_py", "carm.py")
    if not os.path.exists(carm_py):
        raise RuntimeError("vendor carm.py not found: {}".format(carm_py))
    spec = importlib.util.spec_from_file_location("carm_vendor_light", carm_py)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.Carm


def vector_to_string(values):
    return "[" + ", ".join("{:.6g}".format(v) for v in values) + "]"


class PyWsMotionNode:
    def __init__(self):
        self.robot_host = rospy.get_param("~robot_host", "192.168.31.60")
        self.allow_motion = rospy.get_param("~allow_motion", False)
        self.dry_run = rospy.get_param("~dry_run", True)
        self.joint_count = rospy.get_param("~joint_count", 6)
        self.max_jog_delta_rad = rospy.get_param("~max_jog_delta_rad", 0.03)
        self.max_move_delta_rad = rospy.get_param("~max_move_delta_rad", 0.15)
        self.wait_state_timeout_s = rospy.get_param("~wait_state_timeout_s", 3.0)
        self.move_speed = rospy.get_param("~move_speed", 5)
        self.use_sdk_clip = rospy.get_param("~use_sdk_clip", True)

        self.lock = threading.RLock()
        Carm = load_vendor_carm_class()
        rospy.logwarn("py_ws_motion loading pure Python WebSocket SDK for %s:8090", self.robot_host)
        self.arm = Carm(addr=self.robot_host)

        rospy.Service("/carm_a3/py_motion/status", Trigger, self.handle_status)
        rospy.Service("/carm_a3/py_motion/jog_joint", JogJoint, self.handle_jog_joint)
        rospy.Service("/carm_a3/py_motion/move_joint", MoveJoint, self.handle_move_joint)

        rospy.logwarn(
            "py_ws_motion started with allow_motion=%s dry_run=%s move_speed=%s",
            self.allow_motion,
            self.dry_run,
            self.move_speed,
        )

    def wait_for_state(self):
        deadline = time.time() + self.wait_state_timeout_s
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.arm.state is not None and "arm" in self.arm.state:
                return True
            time.sleep(0.02)
        return False

    def current_arm_state(self):
        if not self.wait_for_state():
            raise RuntimeError("timed out waiting for WebSocket robot state")
        arms = self.arm.state.get("arm", [])
        if len(arms) <= self.arm.arm_index:
            raise RuntimeError("robot state does not contain arm index {}".format(self.arm.arm_index))
        return arms[self.arm.arm_index]

    def status_text(self):
        arm = self.current_arm_state()
        return (
            "connected={connected},servo={servo},state={state},fsm_state={fsm},"
            "speed_percentage={speed},dof={dof},name={name}"
        ).format(
            connected=self.arm.is_connected(),
            servo=arm.get("servo", arm.get("servo_status", "unknown")),
            state=arm.get("state", "unknown"),
            fsm=arm.get("fsm_state", "unknown"),
            speed=arm.get("speed_percentage", "unknown"),
            dof=arm.get("arm_dof", arm.get("dof", "unknown")),
            name=arm.get("arm_name", arm.get("name", "unknown")),
        )

    def current_joints(self):
        joints = list(self.arm.joint_pos)
        if len(joints) < self.joint_count:
            raise RuntimeError("SDK returned too few joints: {}".format(len(joints)))
        return joints[: self.joint_count]

    def check_motion_gate(self):
        if not self.allow_motion:
            return "blocked: allow_motion is false"
        if not self.arm.is_connected():
            return "blocked: WebSocket is not connected"
        arm = self.current_arm_state()
        if arm.get("state", 0) == -1:
            return "blocked: controller state is error"
        if arm.get("servo", 0) != 1:
            return "blocked: servo is not enabled"
        if arm.get("fsm_state", "") != "POSITION":
            return "blocked: controller is not in POSITION mode"
        return None

    def move_joint_ws(self, target):
        # Use the same JSON command as vendor Carm.move_joint, but expose speed as a conservative param.
        if self.use_sdk_clip:
            return self.arm.move_joint(target, is_sync=False)
        return self.arm.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVJ",
            "task_level": "Task_General",
            "arm_index": self.arm.arm_index,
            "point_type": {"space": 0},
            "data": {"tool": 0, "target_pos": target, "speed": self.move_speed},
        })

    def handle_status(self, _req):
        with self.lock:
            try:
                return TriggerResponse(True, self.status_text())
            except Exception as exc:
                return TriggerResponse(False, str(exc))

    def handle_jog_joint(self, req):
        with self.lock:
            try:
                gate = self.check_motion_gate()
                if gate:
                    return JogJointResponse(False, gate)
                if req.joint_index < 1 or req.joint_index > self.joint_count:
                    return JogJointResponse(False, "joint_index must be in [1, {}]".format(self.joint_count))
                if math.fabs(req.delta_rad) > self.max_jog_delta_rad:
                    return JogJointResponse(False, "abs(delta_rad) exceeds max_jog_delta_rad")

                current = self.current_joints()
                target = list(current)
                target[req.joint_index - 1] += req.delta_rad
                if self.dry_run:
                    return JogJointResponse(
                        True,
                        "dry_run py jog accepted: current={}, target={}".format(
                            vector_to_string(current), vector_to_string(target)
                        ),
                    )

                rospy.logwarn("py_ws_motion calling WebSocket TASK_MOVJ target=%s", vector_to_string(target))
                res = self.move_joint_ws(target)
                rospy.logwarn("py_ws_motion TASK_MOVJ response=%s", res)
                return JogJointResponse(True, "TASK_MOVJ response={}".format(res))
            except Exception as exc:
                return JogJointResponse(False, str(exc))

    def handle_move_joint(self, req):
        with self.lock:
            try:
                gate = self.check_motion_gate()
                if gate:
                    return MoveJointResponse(False, gate)
                if len(req.positions) != self.joint_count:
                    return MoveJointResponse(False, "positions must contain exactly {} values".format(self.joint_count))
                current = self.current_joints()
                for index, (cur, target) in enumerate(zip(current, req.positions), start=1):
                    if math.fabs(target - cur) > self.max_move_delta_rad:
                        return MoveJointResponse(False, "joint {} delta exceeds max_move_delta_rad".format(index))

                target = list(req.positions)
                if self.dry_run:
                    return MoveJointResponse(
                        True,
                        "dry_run py move accepted: current={}, target={}".format(
                            vector_to_string(current), vector_to_string(target)
                        ),
                    )

                rospy.logwarn("py_ws_motion calling WebSocket TASK_MOVJ target=%s", vector_to_string(target))
                res = self.move_joint_ws(target)
                rospy.logwarn("py_ws_motion TASK_MOVJ response=%s", res)
                return MoveJointResponse(True, "TASK_MOVJ response={}".format(res))
            except Exception as exc:
                return MoveJointResponse(False, str(exc))


if __name__ == "__main__":
    rospy.init_node("carm_a3_py_ws_motion")
    try:
        PyWsMotionNode()
        rospy.spin()
    except ImportError as exc:
        rospy.logfatal("Failed to import websocket-client dependency: %s", exc)
        rospy.logfatal("Install it in Ubuntu with: pip3 install --user websocket-client")
        sys.exit(1)
    except Exception as exc:
        rospy.logfatal("py_ws_motion failed: %s", exc)
        sys.exit(1)

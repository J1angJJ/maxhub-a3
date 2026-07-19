#!/usr/bin/env python3
import json
import math
import threading
import time
import uuid

import rospy
import websocket
from std_srvs.srv import Trigger, TriggerResponse

from carm_a3_motion.srv import JogJoint, JogJointResponse
from carm_a3_motion.srv import MoveJoint, MoveJointResponse


def vector_to_string(values):
    return "[" + ", ".join("{:.6g}".format(v) for v in values) + "]"


class RawCarmWsClient:
    def __init__(self, host, port=8090):
        self.host = host
        self.port = port
        self.ws = None
        self.state = None
        self.last_msg = None
        self.connected = threading.Event()
        self.lock = threading.RLock()
        self.pending = {}
        self.reader_thread = None

    def connect(self):
        url = "ws://{}:{}".format(self.host, self.port)
        self.ws = websocket.WebSocketApp(
            url,
            on_open=self._on_open,
            on_close=self._on_close,
            on_message=self._on_message,
            on_error=self._on_error,
        )
        self.reader_thread = threading.Thread(target=self.ws.run_forever, daemon=True)
        self.reader_thread.start()

    def is_connected(self):
        return self.connected.is_set()

    def close(self):
        if self.ws:
            self.ws.close()

    def wait_connected(self, timeout):
        return self.connected.wait(timeout)

    def wait_state(self, timeout):
        deadline = time.time() + timeout
        while not rospy.is_shutdown() and time.time() < deadline:
            with self.lock:
                if self.state is not None and "arm" in self.state:
                    return True
            time.sleep(0.02)
        return False

    def request(self, payload, timeout=3.0):
        task_key = str(uuid.uuid4())
        req = dict(payload)
        req["task_key"] = task_key
        event = threading.Event()
        with self.lock:
            self.pending[task_key] = {"event": event, "response": None}
        self.ws.send(json.dumps(req))
        if not event.wait(timeout):
            with self.lock:
                self.pending.pop(task_key, None)
            raise RuntimeError("request timed out: {}".format(req.get("command", "")))
        with self.lock:
            data = self.pending.pop(task_key)
            return data["response"]

    def move_joint(self, target, speed=5, timeout=3.0):
        return self.request({
            "command": "webRecieveTasks",
            "task_id": "TASK_MOVJ",
            "task_level": "Task_General",
            "arm_index": 0,
            "point_type": {"space": 0},
            "data": {"tool": 0, "target_pos": target, "speed": speed},
        }, timeout=timeout)

    def _on_open(self, _ws):
        rospy.logwarn("raw_ws_motion connected to ws://%s:%s", self.host, self.port)
        self.connected.set()

    def _on_close(self, _ws, code, msg):
        rospy.logwarn("raw_ws_motion disconnected code=%s msg=%s", code, msg)
        self.connected.clear()

    def _on_error(self, _ws, error):
        rospy.logerr("raw_ws_motion websocket error: %s", error)

    def _on_message(self, _ws, message):
        msg = json.loads(message)
        with self.lock:
            self.last_msg = msg
            if msg.get("command") == "webSendRobotState" and "arm" in msg:
                self.state = msg
            task_key = msg.get("task_key")
            if task_key in self.pending:
                self.pending[task_key]["response"] = msg
                self.pending[task_key]["event"].set()


class RawWsMotionNode:
    def __init__(self):
        self.robot_host = rospy.get_param("~robot_host", "192.168.31.60")
        self.robot_port = rospy.get_param("~robot_port", 8090)
        self.allow_motion = rospy.get_param("~allow_motion", False)
        self.dry_run = rospy.get_param("~dry_run", True)
        self.joint_count = rospy.get_param("~joint_count", 6)
        self.max_jog_delta_rad = rospy.get_param("~max_jog_delta_rad", 0.03)
        self.max_move_delta_rad = rospy.get_param("~max_move_delta_rad", 0.15)
        self.wait_state_timeout_s = rospy.get_param("~wait_state_timeout_s", 3.0)
        self.request_timeout_s = rospy.get_param("~request_timeout_s", 3.0)
        self.move_speed = rospy.get_param("~move_speed", 5)

        self.lock = threading.RLock()
        self.client = RawCarmWsClient(self.robot_host, self.robot_port)
        self.client.connect()
        if not self.client.wait_connected(3.0):
            raise RuntimeError("timed out connecting to {}:{}".format(self.robot_host, self.robot_port))

        rospy.Service("/carm_a3/raw_motion/status", Trigger, self.handle_status)
        rospy.Service("/carm_a3/raw_motion/jog_joint", JogJoint, self.handle_jog_joint)
        rospy.Service("/carm_a3/raw_motion/move_joint", MoveJoint, self.handle_move_joint)

        rospy.logwarn(
            "raw_ws_motion started with allow_motion=%s dry_run=%s move_speed=%s",
            self.allow_motion,
            self.dry_run,
            self.move_speed,
        )

    def arm_state(self):
        if not self.client.wait_state(self.wait_state_timeout_s):
            raise RuntimeError("timed out waiting for WebSocket robot state")
        with self.client.lock:
            arms = self.client.state.get("arm", [])
            if not arms:
                raise RuntimeError("robot state contains no arm entries")
            return dict(arms[0])

    def current_joints(self):
        arm = self.arm_state()
        reality = arm.get("reality", {})
        joints = reality.get("pose", [])
        if len(joints) < self.joint_count:
            raise RuntimeError("state returned too few joints: {}".format(len(joints)))
        return list(joints[: self.joint_count])

    def status_text(self):
        arm = self.arm_state()
        return (
            "connected={connected},servo={servo},state={state},fsm_state={fsm},"
            "speed_percentage={speed},dof={dof},name={name}"
        ).format(
            connected=self.client.is_connected(),
            servo=arm.get("servo", "unknown"),
            state=arm.get("state", "unknown"),
            fsm=arm.get("fsm_state", "unknown"),
            speed=arm.get("speed_percentage", "unknown"),
            dof=arm.get("arm_dof", arm.get("dof", "unknown")),
            name=arm.get("arm_name", arm.get("name", "unknown")),
        )

    def check_motion_gate(self):
        if not self.allow_motion:
            return "blocked: allow_motion is false"
        if not self.client.is_connected():
            return "blocked: WebSocket is not connected"
        arm = self.arm_state()
        if arm.get("state", 0) == -1:
            return "blocked: controller state is error"
        if arm.get("servo", 0) != 1:
            return "blocked: servo is not enabled"
        if arm.get("fsm_state", "") != "POSITION":
            return "blocked: controller is not in POSITION mode"
        return None

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
                        "dry_run raw jog accepted: current={}, target={}".format(
                            vector_to_string(current), vector_to_string(target)
                        ),
                    )
                rospy.logwarn("raw_ws_motion sending TASK_MOVJ target=%s", vector_to_string(target))
                res = self.client.move_joint(target, speed=self.move_speed, timeout=self.request_timeout_s)
                rospy.logwarn("raw_ws_motion TASK_MOVJ response=%s", res)
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
                target = list(req.positions)
                for index, (cur, dest) in enumerate(zip(current, target), start=1):
                    if math.fabs(dest - cur) > self.max_move_delta_rad:
                        return MoveJointResponse(False, "joint {} delta exceeds max_move_delta_rad".format(index))
                if self.dry_run:
                    return MoveJointResponse(
                        True,
                        "dry_run raw move accepted: current={}, target={}".format(
                            vector_to_string(current), vector_to_string(target)
                        ),
                    )
                rospy.logwarn("raw_ws_motion sending TASK_MOVJ target=%s", vector_to_string(target))
                res = self.client.move_joint(target, speed=self.move_speed, timeout=self.request_timeout_s)
                rospy.logwarn("raw_ws_motion TASK_MOVJ response=%s", res)
                return MoveJointResponse(True, "TASK_MOVJ response={}".format(res))
            except Exception as exc:
                return MoveJointResponse(False, str(exc))


if __name__ == "__main__":
    rospy.init_node("carm_a3_raw_ws_motion")
    try:
        RawWsMotionNode()
        rospy.spin()
    except ImportError as exc:
        rospy.logfatal("Failed to import websocket-client dependency: %s", exc)
        rospy.logfatal("Install it in Ubuntu with: pip3 install --user websocket-client")
        raise
    except Exception as exc:
        rospy.logfatal("raw_ws_motion failed: %s", exc)
        raise

import time

import gymnasium as gym
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from gymnasium import spaces
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from carm_rl_env.kinematics import ARM_JOINT_NAMES, JOINT_LOWER, JOINT_UPPER, forward_tcp_position


class CArmA3GazeboReachingEnv(gym.Env):
    """Gazebo-backed reaching env.

    This first version expects `ros2 launch carm_gazebo spawn_a3_control.launch.py`
    to already be running. It publishes position trajectories and observes
    `/joint_states`; it does not reset Gazebo physics yet.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        max_steps=50,
        action_scale=0.03,
        command_duration=0.2,
        command_settle_time=0.05,
        command_timeout=None,
        joint_target_tolerance=0.02,
        success_threshold=0.03,
        distance_reward_scale=1.0,
        progress_reward_scale=0.0,
        action_penalty_scale=0.01,
        smoothness_penalty_scale=0.0,
        joint_limit_penalty_scale=0.0,
        success_bonus=0.0,
        target_position=None,
        target_low=None,
        target_high=None,
        hard_target_position=None,
        hard_target_ratio=0.0,
        hard_target_noise=0.03,
        reset_noise=0.0,
        reset_world_on_reset=False,
        node_name="carm_a3_gazebo_reaching_env",
    ):
        super().__init__()
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = Node(node_name)
        self.joint_names = tuple(ARM_JOINT_NAMES)
        self.max_steps = int(max_steps)
        self.action_scale = float(action_scale)
        self.command_duration = float(command_duration)
        self.command_settle_time = float(command_settle_time)
        if command_timeout is None:
            command_timeout = self.command_duration + self.command_settle_time
        self.command_timeout = float(command_timeout)
        self.joint_target_tolerance = float(joint_target_tolerance)
        self.success_threshold = float(success_threshold)
        self.distance_reward_scale = float(distance_reward_scale)
        self.progress_reward_scale = float(progress_reward_scale)
        self.action_penalty_scale = float(action_penalty_scale)
        self.smoothness_penalty_scale = float(smoothness_penalty_scale)
        self.joint_limit_penalty_scale = float(joint_limit_penalty_scale)
        self.success_bonus = float(success_bonus)
        self.reset_noise = float(reset_noise)
        self.reset_world_on_reset = bool(reset_world_on_reset)
        self.fixed_target_position = None
        if target_position is not None:
            self.fixed_target_position = np.asarray(target_position, dtype=np.float32)
        self.hard_target_position = None
        if hard_target_position is not None:
            self.hard_target_position = np.asarray(hard_target_position, dtype=np.float32)
        self.hard_target_ratio = float(hard_target_ratio)
        self.hard_target_noise = float(hard_target_noise)

        observation_target_low = np.array([0.15, -0.25, 0.10], dtype=np.float32)
        observation_target_high = np.array([0.55, 0.25, 0.55], dtype=np.float32)
        self.target_low = observation_target_low.copy()
        self.target_high = observation_target_high.copy()
        if target_low is not None:
            self.target_low = np.asarray(target_low, dtype=np.float32)
        if target_high is not None:
            self.target_high = np.asarray(target_high, dtype=np.float32)
        if np.any(self.target_low >= self.target_high):
            raise ValueError("target_low must be smaller than target_high on every axis.")
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)
        tcp_low = np.array([-1.0, -1.0, -0.5], dtype=np.float32)
        tcp_high = np.array([1.0, 1.0, 1.0], dtype=np.float32)
        delta_low = observation_target_low - tcp_high
        delta_high = observation_target_high - tcp_low
        obs_low = np.concatenate([JOINT_LOWER, tcp_low, observation_target_low, delta_low]).astype(np.float32)
        obs_high = np.concatenate([JOINT_UPPER, tcp_high, observation_target_high, delta_high]).astype(np.float32)
        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float32)

        self.publisher = self.node.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.subscription = self.node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )
        self.reset_world_client = None
        if self.reset_world_on_reset:
            self.reset_world_client = self.node.create_client(Empty, "/reset_world")
        self._latest_joint_positions = None
        self._step_count = 0
        self.previous_action = np.zeros(6, dtype=np.float32)
        self.previous_distance = None
        self.target_position = np.zeros(3, dtype=np.float32)

    def _on_joint_state(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        try:
            self._latest_joint_positions = np.array([by_name[name] for name in self.joint_names], dtype=np.float32)
        except KeyError:
            return

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    def _wait_for_joint_positions(self, timeout_sec=5.0):
        if self._latest_joint_positions is not None and timeout_sec <= 0.0:
            return self._latest_joint_positions.copy()
        ok = self._spin_until(lambda: self._latest_joint_positions is not None, timeout_sec)
        if not ok:
            raise TimeoutError("Timed out waiting for /joint_states. Start carm_gazebo spawn_a3_control.launch.py first.")
        return self._latest_joint_positions.copy()

    def _publish_joint_target(self, positions):
        msg = JointTrajectory()
        msg.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        sec = int(self.command_duration)
        nanosec = int((self.command_duration - sec) * 1_000_000_000)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(point)
        self.publisher.publish(msg)

    def _sleep_spin(self, duration_sec):
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _call_reset_world(self):
        if self.reset_world_client is None:
            return False
        if not self.reset_world_client.wait_for_service(timeout_sec=2.0):
            raise TimeoutError("Timed out waiting for /reset_world. Start Gazebo first.")
        future = self.reset_world_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        if not future.done():
            raise TimeoutError("Timed out calling /reset_world.")
        future.result()
        self._latest_joint_positions = None
        self._sleep_spin(0.05)
        return True

    def _wait_for_command(self, target_joints):
        self._sleep_spin(self.command_settle_time)
        target_joints = np.asarray(target_joints, dtype=np.float32)
        deadline = time.monotonic() + max(0.0, self.command_timeout - self.command_settle_time)
        latest = self._wait_for_joint_positions()
        while time.monotonic() < deadline:
            error = np.abs(latest - target_joints)
            if float(np.max(error)) <= self.joint_target_tolerance:
                return latest, float(np.max(error)), True
            rclpy.spin_once(self.node, timeout_sec=0.02)
            latest = self._wait_for_joint_positions(timeout_sec=0.0)
        error = np.abs(latest - target_joints)
        return latest, float(np.max(error)), False

    def _sample_target(self):
        if self.fixed_target_position is not None:
            return self.fixed_target_position.copy()
        if self.hard_target_position is not None and self.np_random.random() < self.hard_target_ratio:
            noise = self.np_random.uniform(-self.hard_target_noise, self.hard_target_noise, size=3).astype(np.float32)
            return np.clip(self.hard_target_position + noise, self.target_low, self.target_high).astype(np.float32)
        return self.np_random.uniform(low=self.target_low, high=self.target_high).astype(np.float32)

    def _joint_limit_penalty(self, joint_positions):
        span = JOINT_UPPER - JOINT_LOWER
        margin = np.minimum(joint_positions - JOINT_LOWER, JOINT_UPPER - joint_positions) / span
        limit_violation = np.maximum(0.0, 0.10 - margin) / 0.10
        return float(np.mean(limit_violation))

    def _get_obs(self, joint_positions):
        tcp_position = forward_tcp_position(joint_positions)
        delta = self.target_position - tcp_position
        return np.concatenate([
            joint_positions.astype(np.float32),
            tcp_position,
            self.target_position,
            delta,
        ]).astype(np.float32)

    def _get_info(self, joint_positions):
        tcp_position = forward_tcp_position(joint_positions)
        distance = float(np.linalg.norm(self.target_position - tcp_position))
        return {
            "tcp_position": tcp_position,
            "target_position": self.target_position.copy(),
            "distance": distance,
            "step_count": self._step_count,
        }

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._step_count = 0
        self.previous_action = np.zeros(6, dtype=np.float32)
        self.previous_distance = None
        options = options or {}
        gazebo_reset_called = self._call_reset_world() if self.reset_world_on_reset else False
        current = self._wait_for_joint_positions()

        if "joint_positions" in options:
            target_joints = np.asarray(options["joint_positions"], dtype=np.float32)
        else:
            neutral = 0.5 * (JOINT_LOWER + JOINT_UPPER)
            noise = self.np_random.uniform(-self.reset_noise, self.reset_noise, size=6).astype(np.float32)
            neutral = neutral + noise
            target_joints = np.clip(neutral, JOINT_LOWER, JOINT_UPPER)

        self._publish_joint_target(target_joints)
        current, joint_target_error, joint_target_reached = self._wait_for_command(target_joints)

        if "target_position" in options:
            self.target_position = np.asarray(options["target_position"], dtype=np.float32)
        else:
            self.target_position = self._sample_target()

        info = self._get_info(current)
        self.previous_distance = info["distance"]
        info["joint_target_error"] = joint_target_error
        info["joint_target_reached"] = joint_target_reached
        info["gazebo_reset_called"] = gazebo_reset_called
        return self._get_obs(current), info

    def step(self, action):
        current = self._wait_for_joint_positions()
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        target_joints = np.clip(current + action * self.action_scale, JOINT_LOWER, JOINT_UPPER).astype(np.float32)
        self._publish_joint_target(target_joints)
        current, joint_target_error, joint_target_reached = self._wait_for_command(target_joints)
        self._step_count += 1

        info = self._get_info(current)
        distance = info["distance"]
        previous_distance = self.previous_distance if self.previous_distance is not None else distance
        progress_reward = self.progress_reward_scale * (previous_distance - distance)
        action_penalty = self.action_penalty_scale * float(np.linalg.norm(action))
        smoothness_penalty = self.smoothness_penalty_scale * float(np.linalg.norm(action - self.previous_action))
        joint_limit_penalty = self.joint_limit_penalty_scale * self._joint_limit_penalty(current)
        terminated = distance < self.success_threshold
        reward = (
            -self.distance_reward_scale * distance
            + progress_reward
            - action_penalty
            - smoothness_penalty
            - joint_limit_penalty
            + (self.success_bonus if terminated else 0.0)
        )
        truncated = self._step_count >= self.max_steps
        info["progress_reward"] = progress_reward
        info["action_penalty"] = action_penalty
        info["smoothness_penalty"] = smoothness_penalty
        info["joint_limit_penalty"] = joint_limit_penalty
        info["commanded_joint_positions"] = target_joints.copy()
        info["joint_target_error"] = joint_target_error
        info["joint_target_reached"] = joint_target_reached
        self.previous_action = action.copy()
        self.previous_distance = distance
        return self._get_obs(current), float(reward), terminated, truncated, info

    def close(self):
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
            self.node = None

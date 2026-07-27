import time

import gymnasium as gym
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from gymnasium import spaces
from rclpy.node import Node
from sensor_msgs.msg import JointState
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
        success_threshold=0.03,
        distance_reward_scale=1.0,
        action_penalty_scale=0.01,
        target_position=None,
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
        self.success_threshold = float(success_threshold)
        self.distance_reward_scale = float(distance_reward_scale)
        self.action_penalty_scale = float(action_penalty_scale)
        self.fixed_target_position = None
        if target_position is not None:
            self.fixed_target_position = np.asarray(target_position, dtype=np.float32)

        self.target_low = np.array([0.15, -0.25, 0.10], dtype=np.float32)
        self.target_high = np.array([0.55, 0.25, 0.55], dtype=np.float32)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(6,), dtype=np.float32)
        tcp_low = np.array([-1.0, -1.0, -0.5], dtype=np.float32)
        tcp_high = np.array([1.0, 1.0, 1.0], dtype=np.float32)
        delta_low = self.target_low - tcp_high
        delta_high = self.target_high - tcp_low
        obs_low = np.concatenate([JOINT_LOWER, tcp_low, self.target_low, delta_low]).astype(np.float32)
        obs_high = np.concatenate([JOINT_UPPER, tcp_high, self.target_high, delta_high]).astype(np.float32)
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
        self._latest_joint_positions = None
        self._step_count = 0
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

    def _sample_target(self):
        if self.fixed_target_position is not None:
            return self.fixed_target_position.copy()
        return self.np_random.uniform(low=self.target_low, high=self.target_high).astype(np.float32)

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
        options = options or {}
        current = self._wait_for_joint_positions()

        if "joint_positions" in options:
            target_joints = np.asarray(options["joint_positions"], dtype=np.float32)
        else:
            neutral = 0.5 * (JOINT_LOWER + JOINT_UPPER)
            target_joints = np.clip(neutral, JOINT_LOWER, JOINT_UPPER)

        self._publish_joint_target(target_joints)
        self._sleep_spin(max(self.command_duration, 0.05) + self.command_settle_time)
        current = self._wait_for_joint_positions()

        if "target_position" in options:
            self.target_position = np.asarray(options["target_position"], dtype=np.float32)
        else:
            self.target_position = self._sample_target()

        return self._get_obs(current), self._get_info(current)

    def step(self, action):
        current = self._wait_for_joint_positions()
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        target_joints = np.clip(current + action * self.action_scale, JOINT_LOWER, JOINT_UPPER).astype(np.float32)
        self._publish_joint_target(target_joints)
        self._sleep_spin(self.command_duration + self.command_settle_time)
        current = self._wait_for_joint_positions()
        self._step_count += 1

        info = self._get_info(current)
        distance = info["distance"]
        action_penalty = self.action_penalty_scale * float(np.linalg.norm(action))
        terminated = distance < self.success_threshold
        reward = -self.distance_reward_scale * distance - action_penalty
        truncated = self._step_count >= self.max_steps
        info["action_penalty"] = action_penalty
        info["commanded_joint_positions"] = target_joints.copy()
        return self._get_obs(current), float(reward), terminated, truncated, info

    def close(self):
        if getattr(self, "node", None) is not None:
            self.node.destroy_node()
            self.node = None

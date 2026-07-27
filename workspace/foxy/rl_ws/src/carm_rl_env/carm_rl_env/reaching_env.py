import gymnasium as gym
import numpy as np
from gymnasium import spaces

from carm_rl_env.kinematics import JOINT_LOWER, JOINT_UPPER, forward_tcp_position


class CArmA3ReachingEnv(gym.Env):
    """Minimal reaching task with a Gymnasium API.

    This is a lightweight baseline environment. It uses URDF-derived joint
    limits and a simple FK helper, but does not simulate dynamics, contacts, or
    actuator delay yet.
    """

    metadata = {"render_modes": []}

    def __init__(
        self,
        max_steps=100,
        action_scale=0.05,
        success_threshold=0.03,
        distance_reward_scale=1.0,
        action_penalty_scale=0.01,
        smoothness_penalty_scale=0.0,
        joint_limit_penalty_scale=0.0,
        success_bonus=0.0,
        target_position=None,
        hard_target_position=None,
        hard_target_ratio=0.0,
        hard_target_noise=0.03,
    ):
        super().__init__()
        self.max_steps = int(max_steps)
        self.action_scale = float(action_scale)
        self.success_threshold = float(success_threshold)
        self.distance_reward_scale = float(distance_reward_scale)
        self.action_penalty_scale = float(action_penalty_scale)
        self.smoothness_penalty_scale = float(smoothness_penalty_scale)
        self.joint_limit_penalty_scale = float(joint_limit_penalty_scale)
        self.success_bonus = float(success_bonus)
        self.fixed_target_position = None
        if target_position is not None:
            self.fixed_target_position = np.asarray(target_position, dtype=np.float32)
        self.hard_target_position = None
        if hard_target_position is not None:
            self.hard_target_position = np.asarray(hard_target_position, dtype=np.float32)
        self.hard_target_ratio = float(hard_target_ratio)
        self.hard_target_noise = float(hard_target_noise)
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

        self._step_count = 0
        self.joint_positions = np.zeros(6, dtype=np.float32)
        self.previous_action = np.zeros(6, dtype=np.float32)
        self.target_position = np.zeros(3, dtype=np.float32)

    def _joint_limit_penalty(self):
        span = JOINT_UPPER - JOINT_LOWER
        margin = np.minimum(self.joint_positions - JOINT_LOWER, JOINT_UPPER - self.joint_positions) / span
        limit_violation = np.maximum(0.0, 0.10 - margin) / 0.10
        return float(np.mean(limit_violation))

    def _sample_target(self):
        if self.fixed_target_position is not None:
            return self.fixed_target_position.copy()
        if self.hard_target_position is not None and self._rng.random() < self.hard_target_ratio:
            noise = self._rng.uniform(-self.hard_target_noise, self.hard_target_noise, size=3).astype(np.float32)
            return np.clip(self.hard_target_position + noise, self.target_low, self.target_high).astype(np.float32)
        return self._rng.uniform(low=self.target_low, high=self.target_high).astype(np.float32)

    def _get_obs(self):
        tcp_position = forward_tcp_position(self.joint_positions)
        delta = self.target_position - tcp_position
        return np.concatenate([
            self.joint_positions.astype(np.float32),
            tcp_position,
            self.target_position,
            delta,
        ]).astype(np.float32)

    def _get_info(self):
        tcp_position = forward_tcp_position(self.joint_positions)
        distance = float(np.linalg.norm(self.target_position - tcp_position))
        return {
            "tcp_position": tcp_position,
            "target_position": self.target_position.copy(),
            "distance": distance,
            "step_count": self._step_count,
        }

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._rng = self.np_random
        self._step_count = 0
        self.previous_action = np.zeros(6, dtype=np.float32)

        options = options or {}
        if "joint_positions" in options:
            self.joint_positions = np.asarray(options["joint_positions"], dtype=np.float32)
        else:
            neutral = 0.5 * (JOINT_LOWER + JOINT_UPPER)
            noise = self._rng.uniform(-0.05, 0.05, size=6).astype(np.float32)
            self.joint_positions = np.clip(neutral + noise, JOINT_LOWER, JOINT_UPPER)

        if "target_position" in options:
            self.target_position = np.asarray(options["target_position"], dtype=np.float32)
        else:
            self.target_position = self._sample_target()

        return self._get_obs(), self._get_info()

    def step(self, action):
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)
        self.joint_positions = np.clip(
            self.joint_positions + action * self.action_scale,
            JOINT_LOWER,
            JOINT_UPPER,
        ).astype(np.float32)
        self._step_count += 1

        info = self._get_info()
        distance = info["distance"]
        action_penalty = self.action_penalty_scale * float(np.linalg.norm(action))
        smoothness_penalty = self.smoothness_penalty_scale * float(np.linalg.norm(action - self.previous_action))
        joint_limit_penalty = self.joint_limit_penalty_scale * self._joint_limit_penalty()
        terminated = distance < self.success_threshold
        reward = (
            -self.distance_reward_scale * distance
            - action_penalty
            - smoothness_penalty
            - joint_limit_penalty
            + (self.success_bonus if terminated else 0.0)
        )
        truncated = self._step_count >= self.max_steps
        info["action_penalty"] = action_penalty
        info["smoothness_penalty"] = smoothness_penalty
        info["joint_limit_penalty"] = joint_limit_penalty
        self.previous_action = action.copy()

        return self._get_obs(), float(reward), terminated, truncated, info

import argparse
import csv
from pathlib import Path

from stable_baselines3 import A2C, PPO

from carm_rl_env.cli_utils import parse_target_position
from carm_rl_gazebo.gazebo_reaching_env import CArmA3GazeboReachingEnv


ALGORITHMS = {
    "ppo": PPO,
    "a2c": A2C,
}


def _row(step, action, reward, terminated, truncated, info, obs):
    tcp = info["tcp_position"]
    target = info["target_position"]
    commanded = info["commanded_joint_positions"]
    joints = obs[:6]
    return {
        "step": step,
        "reward": reward,
        "distance": info["distance"],
        "best_distance": info.get("best_distance", info["distance"]),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "progress_reward": info.get("progress_reward", 0.0),
        "distance_regression_penalty": info.get("distance_regression_penalty", 0.0),
        "joint_target_error": info.get("joint_target_error", 0.0),
        "joint_target_reached": info.get("joint_target_reached", False),
        "gazebo_reset_called": info.get("gazebo_reset_called", False),
        "target_x": float(target[0]),
        "target_y": float(target[1]),
        "target_z": float(target[2]),
        "tcp_x": float(tcp[0]),
        "tcp_y": float(tcp[1]),
        "tcp_z": float(tcp[2]),
        "action_1": float(action[0]),
        "action_2": float(action[1]),
        "action_3": float(action[2]),
        "action_4": float(action[3]),
        "action_5": float(action[4]),
        "action_6": float(action[5]),
        "joint_1": float(joints[0]),
        "joint_2": float(joints[1]),
        "joint_3": float(joints[2]),
        "joint_4": float(joints[3]),
        "joint_5": float(joints[4]),
        "joint_6": float(joints[5]),
        "commanded_joint_1": float(commanded[0]),
        "commanded_joint_2": float(commanded[1]),
        "commanded_joint_3": float(commanded[2]),
        "commanded_joint_4": float(commanded[3]),
        "commanded_joint_5": float(commanded[4]),
        "commanded_joint_6": float(commanded[5]),
        "step_count": info["step_count"],
    }


def main():
    parser = argparse.ArgumentParser(description="Trace one Gazebo CArm A3 reaching episode to CSV.")
    parser.add_argument("--model", required=True, help="Path to a saved SB3 .zip model.")
    parser.add_argument("--algo", choices=sorted(ALGORITHMS), default="ppo")
    parser.add_argument("--seed", type=int, default=3000)
    parser.add_argument("--target-position", type=parse_target_position, default=None, help="Fixed target as x,y,z.")
    parser.add_argument("--max-steps", type=int, default=30)
    parser.add_argument("--action-scale", type=float, default=0.03)
    parser.add_argument("--command-duration", type=float, default=0.05)
    parser.add_argument("--command-settle-time", type=float, default=0.01)
    parser.add_argument("--command-timeout", type=float, default=0.05)
    parser.add_argument("--joint-target-tolerance", type=float, default=0.08)
    parser.add_argument("--success-threshold", type=float, default=0.03)
    parser.add_argument("--distance-reward-scale", type=float, default=1.0)
    parser.add_argument("--progress-reward-scale", type=float, default=0.0)
    parser.add_argument(
        "--distance-regression-penalty-scale",
        type=float,
        default=0.0,
        help="Penalty multiplier for steps that move farther from the target than the previous step.",
    )
    parser.add_argument("--action-penalty-scale", type=float, default=0.01)
    parser.add_argument("--smoothness-penalty-scale", type=float, default=0.0)
    parser.add_argument("--joint-limit-penalty-scale", type=float, default=0.0)
    parser.add_argument("--success-bonus", type=float, default=0.0)
    parser.add_argument("--target-low", type=parse_target_position, default=None, help="Target sampling lower bound as x,y,z.")
    parser.add_argument("--target-high", type=parse_target_position, default=None, help="Target sampling upper bound as x,y,z.")
    parser.add_argument("--hard-target-position", type=parse_target_position, default=None, help="Replay target center as x,y,z.")
    parser.add_argument("--hard-target-ratio", type=float, default=0.0)
    parser.add_argument("--hard-target-noise", type=float, default=0.03)
    parser.add_argument("--reset-noise", type=float, default=0.0, help="Uniform joint reset noise in radians.")
    parser.add_argument("--reset-world-on-reset", action="store_true", help="Call /reset_world before env reset.")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--csv", required=True)
    args = parser.parse_args()

    env_kwargs = {
        "max_steps": args.max_steps,
        "action_scale": args.action_scale,
        "command_duration": args.command_duration,
        "command_settle_time": args.command_settle_time,
        "command_timeout": args.command_timeout,
        "joint_target_tolerance": args.joint_target_tolerance,
        "success_threshold": args.success_threshold,
        "distance_reward_scale": args.distance_reward_scale,
        "progress_reward_scale": args.progress_reward_scale,
        "distance_regression_penalty_scale": args.distance_regression_penalty_scale,
        "action_penalty_scale": args.action_penalty_scale,
        "smoothness_penalty_scale": args.smoothness_penalty_scale,
        "joint_limit_penalty_scale": args.joint_limit_penalty_scale,
        "success_bonus": args.success_bonus,
        "target_position": args.target_position,
        "target_low": args.target_low,
        "target_high": args.target_high,
        "hard_target_position": args.hard_target_position,
        "hard_target_ratio": args.hard_target_ratio,
        "hard_target_noise": args.hard_target_noise,
        "reset_noise": args.reset_noise,
        "reset_world_on_reset": args.reset_world_on_reset,
    }

    model = ALGORITHMS[args.algo].load(args.model, device=args.device)
    env = CArmA3GazeboReachingEnv(**env_kwargs)
    try:
        reset_options = {"target_position": args.target_position} if args.target_position is not None else None
        obs, _ = env.reset(seed=args.seed, options=reset_options)

        rows = []
        total_reward = 0.0
        terminated = False
        truncated = False
        while not (terminated or truncated):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            rows.append(_row(len(rows) + 1, action, reward, terminated, truncated, info, obs))
    finally:
        env.close()

    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    target_reached = [row["joint_target_reached"] for row in rows]
    print(f"csv={csv_path}")
    print(f"seed={args.seed}")
    print(f"steps={len(rows)}")
    print(f"total_reward={total_reward:.4f}")
    print(f"final_distance={rows[-1]['distance']:.4f}")
    print(f"best_distance={rows[-1]['best_distance']:.4f}")
    print(f"joint_target_reached_rate={sum(target_reached) / len(target_reached):.4f}")
    print(f"terminated={rows[-1]['terminated']}")
    print(f"truncated={rows[-1]['truncated']}")

import argparse
import csv
from pathlib import Path

from stable_baselines3 import A2C, PPO

from carm_rl_env.cli_utils import parse_target_position
from carm_rl_env.reaching_env import CArmA3ReachingEnv


ALGORITHMS = {
    "ppo": PPO,
    "a2c": A2C,
}


def _run_episode(model, seed, max_steps, success_threshold, target_position):
    env = CArmA3ReachingEnv(max_steps=max_steps, success_threshold=success_threshold)
    reset_options = {"target_position": target_position} if target_position is not None else None
    obs, info = env.reset(seed=seed, options=reset_options)
    total_reward = 0.0
    terminated = False
    truncated = False

    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward

    tcp = info["tcp_position"]
    target = info["target_position"]
    delta = target - tcp
    success = info["distance"] < success_threshold
    return {
        "seed": seed,
        "distance": info["distance"],
        "reward": total_reward,
        "episode_length": info["step_count"],
        "success": bool(success),
        "truncated": bool(truncated),
        "failure_reason": "" if success else ("timeout" if truncated else "distance"),
        "target_x": float(target[0]),
        "target_y": float(target[1]),
        "target_z": float(target[2]),
        "tcp_x": float(tcp[0]),
        "tcp_y": float(tcp[1]),
        "tcp_z": float(tcp[2]),
        "delta_x": float(delta[0]),
        "delta_y": float(delta[1]),
        "delta_z": float(delta[2]),
    }


def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained CArm A3 reaching policy.")
    parser.add_argument("--model", required=True, help="Path to a saved SB3 .zip model.")
    parser.add_argument("--algo", choices=sorted(ALGORITHMS), default="ppo")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2000)
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--success-threshold", type=float, default=0.03)
    parser.add_argument("--target-seed", type=int, default=None, help="Evaluate one target seed repeatedly.")
    parser.add_argument("--target-position", type=parse_target_position, default=None, help="Fixed target as x,y,z.")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--csv", default=None, help="Optional CSV path for per-episode metrics.")
    args = parser.parse_args()

    model = ALGORITHMS[args.algo].load(args.model, device=args.device)
    if args.target_seed is not None:
        seeds = [args.target_seed for _ in range(args.episodes)]
    else:
        seeds = [args.seed + idx for idx in range(args.episodes)]
    rows = [
        _run_episode(model, seed, args.max_steps, args.success_threshold, args.target_position)
        for seed in seeds
    ]

    distances = [row["distance"] for row in rows]
    rewards = [row["reward"] for row in rows]
    lengths = [row["episode_length"] for row in rows]
    successes = [row["success"] for row in rows]

    if args.csv:
        csv_path = Path(args.csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "seed",
                    "distance",
                    "reward",
                    "episode_length",
                    "success",
                    "truncated",
                    "failure_reason",
                    "target_x",
                    "target_y",
                    "target_z",
                    "tcp_x",
                    "tcp_y",
                    "tcp_z",
                    "delta_x",
                    "delta_y",
                    "delta_z",
                ],
            )
            writer.writeheader()
            writer.writerows(rows)
        print(f"csv={csv_path}")

    print(f"model={args.model}")
    print(f"episodes={args.episodes}")
    print(f"success_threshold={args.success_threshold:.4f}")
    print(f"success_rate={sum(successes) / len(successes):.4f}")
    print(f"mean_distance={sum(distances) / len(distances):.4f}")
    print(f"best_distance={min(distances):.4f}")
    print(f"worst_distance={max(distances):.4f}")
    print(f"mean_episode_length={sum(lengths) / len(lengths):.2f}")
    print(f"mean_reward={sum(rewards) / len(rewards):.4f}")
    failures = [row for row in rows if not row["success"]]
    print(f"failure_count={len(failures)}")
    if failures:
        worst = max(failures, key=lambda row: row["distance"])
        print(
            "worst_failure="
            f"seed={worst['seed']} "
            f"distance={worst['distance']:.4f} "
            f"target=({worst['target_x']:.4f},{worst['target_y']:.4f},{worst['target_z']:.4f}) "
            f"tcp=({worst['tcp_x']:.4f},{worst['tcp_y']:.4f},{worst['tcp_z']:.4f})"
        )

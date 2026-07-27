import argparse
import csv
from pathlib import Path

from stable_baselines3 import A2C, PPO

from carm_rl_env.reaching_env import CArmA3ReachingEnv


ALGORITHMS = {
    "ppo": PPO,
    "a2c": A2C,
}


def _run_episode(model, seed, max_steps):
    env = CArmA3ReachingEnv(max_steps=max_steps)
    obs, info = env.reset(seed=seed)
    total_reward = 0.0
    terminated = False
    truncated = False

    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward

    return {
        "seed": seed,
        "distance": info["distance"],
        "reward": total_reward,
        "episode_length": info["step_count"],
        "success": bool(terminated),
        "truncated": bool(truncated),
    }


def main():
    parser = argparse.ArgumentParser(description="Evaluate a trained CArm A3 reaching policy.")
    parser.add_argument("--model", required=True, help="Path to a saved SB3 .zip model.")
    parser.add_argument("--algo", choices=sorted(ALGORITHMS), default="ppo")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2000)
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--success-threshold", type=float, default=0.03)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--csv", default=None, help="Optional CSV path for per-episode metrics.")
    args = parser.parse_args()

    model = ALGORITHMS[args.algo].load(args.model, device=args.device)
    rows = [_run_episode(model, args.seed + idx, args.max_steps) for idx in range(args.episodes)]

    distances = [row["distance"] for row in rows]
    rewards = [row["reward"] for row in rows]
    lengths = [row["episode_length"] for row in rows]
    successes = [row["distance"] < args.success_threshold for row in rows]

    if args.csv:
        csv_path = Path(args.csv)
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["seed", "distance", "reward", "episode_length", "success", "truncated"],
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

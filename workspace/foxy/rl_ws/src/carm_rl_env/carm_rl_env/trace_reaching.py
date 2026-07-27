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


def _row(step, action, reward, terminated, truncated, info, joint_positions):
    tcp = info["tcp_position"]
    target = info["target_position"]
    return {
        "step": step,
        "reward": reward,
        "distance": info["distance"],
        "terminated": bool(terminated),
        "truncated": bool(truncated),
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
        "joint_1": float(joint_positions[0]),
        "joint_2": float(joint_positions[1]),
        "joint_3": float(joint_positions[2]),
        "joint_4": float(joint_positions[3]),
        "joint_5": float(joint_positions[4]),
        "joint_6": float(joint_positions[5]),
        "step_count": info["step_count"],
    }


def main():
    parser = argparse.ArgumentParser(description="Trace one CArm A3 reaching episode to CSV.")
    parser.add_argument("--model", required=True, help="Path to a saved SB3 .zip model.")
    parser.add_argument("--algo", choices=sorted(ALGORITHMS), default="ppo")
    parser.add_argument("--seed", type=int, default=2080)
    parser.add_argument("--target-position", type=parse_target_position, default=None, help="Fixed target as x,y,z.")
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--success-threshold", type=float, default=0.03)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--csv", required=True)
    args = parser.parse_args()

    model = ALGORITHMS[args.algo].load(args.model, device=args.device)
    env = CArmA3ReachingEnv(max_steps=args.max_steps, success_threshold=args.success_threshold)
    reset_options = {"target_position": args.target_position} if args.target_position is not None else None
    obs, info = env.reset(seed=args.seed, options=reset_options)

    rows = []
    total_reward = 0.0
    terminated = False
    truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        rows.append(_row(len(rows) + 1, action, reward, terminated, truncated, info, env.joint_positions))

    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print(f"csv={csv_path}")
    print(f"seed={args.seed}")
    print(f"steps={len(rows)}")
    print(f"total_reward={total_reward:.4f}")
    print(f"final_distance={rows[-1]['distance']:.4f}")
    print(f"terminated={rows[-1]['terminated']}")
    print(f"truncated={rows[-1]['truncated']}")

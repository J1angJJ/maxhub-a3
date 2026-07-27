import argparse

from carm_rl_gazebo.gazebo_reaching_env import CArmA3GazeboReachingEnv


def main():
    parser = argparse.ArgumentParser(description="Run a short random rollout against the Gazebo reaching env.")
    parser.add_argument("--steps", type=int, default=10)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--max-steps", type=int, default=50)
    parser.add_argument("--command-duration", type=float, default=0.2)
    parser.add_argument("--command-settle-time", type=float, default=0.05)
    args = parser.parse_args()

    env = CArmA3GazeboReachingEnv(
        max_steps=args.max_steps,
        command_duration=args.command_duration,
        command_settle_time=args.command_settle_time,
    )
    try:
        obs, info = env.reset(seed=args.seed)
        print(f"reset obs_shape={obs.shape} distance={info['distance']:.4f}")
        total_reward = 0.0
        for step in range(args.steps):
            obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
            total_reward += reward
            print(
                f"step={step + 1:02d} reward={reward:.4f} "
                f"distance={info['distance']:.4f} terminated={terminated} truncated={truncated}"
            )
            if terminated or truncated:
                break
        print(f"total_reward={total_reward:.4f}")
    finally:
        env.close()

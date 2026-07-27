import argparse
from pathlib import Path

from stable_baselines3 import A2C, PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv

from carm_rl_env.cli_utils import parse_target_position
from carm_rl_gazebo.gazebo_reaching_env import CArmA3GazeboReachingEnv


ALGORITHMS = {
    "ppo": PPO,
    "a2c": A2C,
}


def _env_kwargs(args):
    return {
        "max_steps": args.max_steps,
        "action_scale": args.action_scale,
        "command_duration": args.command_duration,
        "command_settle_time": args.command_settle_time,
        "success_threshold": args.success_threshold,
        "distance_reward_scale": args.distance_reward_scale,
        "action_penalty_scale": args.action_penalty_scale,
        "target_position": args.target_position,
    }


def _make_env(env_kwargs, seed):
    def _init():
        env = CArmA3GazeboReachingEnv(**env_kwargs)
        env.reset(seed=seed)
        return env

    return _init


def _evaluate(model, episodes, env_kwargs):
    distances = []
    rewards = []
    env = CArmA3GazeboReachingEnv(**env_kwargs)
    try:
        for episode in range(episodes):
            obs, info = env.reset(seed=1000 + episode)
            total_reward = 0.0
            terminated = False
            truncated = False
            while not (terminated or truncated):
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
            distances.append(info["distance"])
            rewards.append(total_reward)
    finally:
        env.close()
    return distances, rewards


def main():
    parser = argparse.ArgumentParser(description="Train a Gazebo-backed CArm A3 reaching policy.")
    parser.add_argument("--algo", choices=sorted(ALGORITHMS), default="ppo")
    parser.add_argument("--timesteps", type=int, default=256)
    parser.add_argument("--max-steps", type=int, default=30)
    parser.add_argument("--action-scale", type=float, default=0.03)
    parser.add_argument("--command-duration", type=float, default=0.15)
    parser.add_argument("--command-settle-time", type=float, default=0.05)
    parser.add_argument("--success-threshold", type=float, default=0.03)
    parser.add_argument("--distance-reward-scale", type=float, default=1.0)
    parser.add_argument("--action-penalty-scale", type=float, default=0.01)
    parser.add_argument("--target-position", type=parse_target_position, default=None, help="Fixed target as x,y,z.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cpu", help="auto, cpu, cuda, or cuda:0")
    parser.add_argument("--save-dir", default="/workspace/rl_ws/artifacts/gazebo_reaching")
    parser.add_argument("--load-model", default=None, help="Path to an existing SB3 .zip model for continued training.")
    parser.add_argument(
        "--reset-num-timesteps",
        action="store_true",
        help="Reset SB3 timestep counters when loading a model. By default continued training keeps counters.",
    )
    parser.add_argument("--eval-episodes", type=int, default=1)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=64, help="PPO rollout steps, or A2C update steps.")
    parser.add_argument("--batch-size", type=int, default=64, help="PPO minibatch size.")
    parser.add_argument("--check-env", action="store_true")
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    env_kwargs = _env_kwargs(args)
    raw_env = CArmA3GazeboReachingEnv(**env_kwargs)
    try:
        if args.check_env:
            check_env(raw_env, warn=True)
    finally:
        raw_env.close()

    env = DummyVecEnv([_make_env(env_kwargs, args.seed)])
    algo_cls = ALGORITHMS[args.algo]
    model_kwargs = {
        "learning_rate": args.learning_rate,
        "n_steps": args.n_steps,
    }
    if args.algo == "ppo":
        model_kwargs["batch_size"] = args.batch_size

    print(f"algo={args.algo}")
    print("num_envs=1")
    print("vec_env=dummy")
    print(f"max_steps={args.max_steps}")
    print(f"success_threshold={args.success_threshold}")
    print(f"timesteps={args.timesteps}")
    print(f"rollout_batch={args.n_steps}")
    print(f"action_scale={args.action_scale}")
    print(f"command_duration={args.command_duration}")
    print(f"command_settle_time={args.command_settle_time}")
    print(f"distance_reward_scale={args.distance_reward_scale}")
    print(f"action_penalty_scale={args.action_penalty_scale}")
    print(f"target_position={args.target_position}")

    if args.load_model:
        model = algo_cls.load(
            args.load_model,
            env=env,
            device=args.device,
            seed=args.seed,
            verbose=1,
        )
        print(f"loaded_model={args.load_model}")
    else:
        model = algo_cls(
            "MlpPolicy",
            env,
            verbose=1,
            seed=args.seed,
            device=args.device,
            **model_kwargs,
        )

    try:
        model.learn(
            total_timesteps=args.timesteps,
            reset_num_timesteps=args.reset_num_timesteps or not args.load_model,
        )
    finally:
        env.close()

    model_suffix = f"{args.timesteps}_more_steps" if args.load_model else f"{args.timesteps}_steps"
    model_path = save_dir / f"{args.algo}_gazebo_reaching_{model_suffix}"
    model.save(model_path)
    print(f"saved_model={model_path}.zip")

    distances, rewards = _evaluate(model, args.eval_episodes, env_kwargs)
    mean_distance = sum(distances) / len(distances)
    mean_reward = sum(rewards) / len(rewards)
    print(f"eval_episodes={args.eval_episodes}")
    print(f"eval_mean_distance={mean_distance:.4f}")
    print(f"eval_mean_reward={mean_reward:.4f}")

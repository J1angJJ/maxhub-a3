import argparse
from pathlib import Path

from stable_baselines3 import A2C, PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env

from carm_rl_env.cli_utils import parse_target_position
from carm_rl_env.reaching_env import CArmA3ReachingEnv


ALGORITHMS = {
    "ppo": PPO,
    "a2c": A2C,
}


def _env_kwargs(args):
    return {
        "max_steps": args.max_steps,
        "success_threshold": args.success_threshold,
        "distance_reward_scale": args.distance_reward_scale,
        "action_penalty_scale": args.action_penalty_scale,
        "smoothness_penalty_scale": args.smoothness_penalty_scale,
        "joint_limit_penalty_scale": args.joint_limit_penalty_scale,
        "success_bonus": args.success_bonus,
        "target_position": args.target_position,
        "hard_target_position": args.hard_target_position,
        "hard_target_ratio": args.hard_target_ratio,
        "hard_target_noise": args.hard_target_noise,
    }


def _evaluate(model, episodes, env_kwargs):
    distances = []
    rewards = []
    for episode in range(episodes):
        env = CArmA3ReachingEnv(**env_kwargs)
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
    return distances, rewards


def main():
    parser = argparse.ArgumentParser(description="Train a minimal CArm A3 reaching policy.")
    parser.add_argument("--algo", choices=sorted(ALGORITHMS), default="ppo")
    parser.add_argument("--timesteps", type=int, default=10_000)
    parser.add_argument("--max-steps", type=int, default=100)
    parser.add_argument("--success-threshold", type=float, default=0.03)
    parser.add_argument("--distance-reward-scale", type=float, default=1.0)
    parser.add_argument("--action-penalty-scale", type=float, default=0.01)
    parser.add_argument("--smoothness-penalty-scale", type=float, default=0.0)
    parser.add_argument("--joint-limit-penalty-scale", type=float, default=0.0)
    parser.add_argument("--success-bonus", type=float, default=0.0)
    parser.add_argument("--target-position", type=parse_target_position, default=None, help="Fixed target as x,y,z.")
    parser.add_argument("--hard-target-position", type=parse_target_position, default=None, help="Replay target center as x,y,z.")
    parser.add_argument("--hard-target-ratio", type=float, default=0.0)
    parser.add_argument("--hard-target-noise", type=float, default=0.03)
    parser.add_argument("--num-envs", type=int, default=1, help="Number of parallel vectorized env instances.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="auto", help="auto, cpu, cuda, or cuda:0")
    parser.add_argument("--save-dir", default="/workspace/rl_ws/artifacts/reaching")
    parser.add_argument("--load-model", default=None, help="Path to an existing SB3 .zip model for continued training.")
    parser.add_argument(
        "--reset-num-timesteps",
        action="store_true",
        help="Reset SB3 timestep counters when loading a model. By default continued training keeps counters.",
    )
    parser.add_argument("--eval-episodes", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=256, help="PPO rollout steps, or A2C update steps.")
    parser.add_argument("--batch-size", type=int, default=64, help="PPO minibatch size.")
    parser.add_argument("--check-env", action="store_true")
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    env_kwargs = _env_kwargs(args)
    raw_env = CArmA3ReachingEnv(**env_kwargs)
    if args.check_env:
        check_env(raw_env, warn=True)

    env = make_vec_env(
        CArmA3ReachingEnv,
        n_envs=args.num_envs,
        seed=args.seed,
        monitor_dir=str(save_dir / "monitor"),
        env_kwargs=env_kwargs,
    )
    algo_cls = ALGORITHMS[args.algo]
    model_kwargs = {
        "learning_rate": args.learning_rate,
        "n_steps": args.n_steps,
    }
    if args.algo == "ppo":
        model_kwargs["batch_size"] = args.batch_size

    print(f"algo={args.algo}")
    print(f"num_envs={args.num_envs}")
    print(f"max_steps={args.max_steps}")
    print(f"success_threshold={args.success_threshold}")
    print(f"timesteps={args.timesteps}")
    print(f"rollout_batch={args.n_steps * args.num_envs}")
    print(f"distance_reward_scale={args.distance_reward_scale}")
    print(f"action_penalty_scale={args.action_penalty_scale}")
    print(f"smoothness_penalty_scale={args.smoothness_penalty_scale}")
    print(f"joint_limit_penalty_scale={args.joint_limit_penalty_scale}")
    print(f"success_bonus={args.success_bonus}")
    print(f"target_position={args.target_position}")
    print(f"hard_target_position={args.hard_target_position}")
    print(f"hard_target_ratio={args.hard_target_ratio}")
    print(f"hard_target_noise={args.hard_target_noise}")

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
    model.learn(
        total_timesteps=args.timesteps,
        reset_num_timesteps=args.reset_num_timesteps or not args.load_model,
    )

    model_suffix = f"{args.timesteps}_more_steps" if args.load_model else f"{args.timesteps}_steps"
    model_path = save_dir / f"{args.algo}_reaching_{model_suffix}"
    model.save(model_path)
    print(f"saved_model={model_path}.zip")

    distances, rewards = _evaluate(model, args.eval_episodes, env_kwargs)
    mean_distance = sum(distances) / len(distances)
    mean_reward = sum(rewards) / len(rewards)
    print(f"eval_episodes={args.eval_episodes}")
    print(f"eval_mean_distance={mean_distance:.4f}")
    print(f"eval_mean_reward={mean_reward:.4f}")

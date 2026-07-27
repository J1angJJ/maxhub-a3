import argparse
from pathlib import Path

from stable_baselines3 import A2C, PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.vec_env import DummyVecEnv

from carm_rl_env.cli_utils import parse_target_position, parse_target_positions
from carm_rl_gazebo.gazebo_reaching_env import CArmA3GazeboReachingEnv


ALGORITHMS = {
    "ppo": PPO,
    "a2c": A2C,
}


def _env_kwargs(args):
    return {
        "max_steps": args.max_steps,
        "action_scale": args.action_scale,
        "near_target_action_scale_radius": args.near_target_action_scale_radius,
        "near_target_action_scale_min": args.near_target_action_scale_min,
        "command_duration": args.command_duration,
        "command_settle_time": args.command_settle_time,
        "command_timeout": args.command_timeout,
        "joint_target_tolerance": args.joint_target_tolerance,
        "success_threshold": args.success_threshold,
        "distance_reward_scale": args.distance_reward_scale,
        "progress_reward_scale": args.progress_reward_scale,
        "distance_regression_penalty_scale": args.distance_regression_penalty_scale,
        "action_penalty_scale": args.action_penalty_scale,
        "near_target_action_penalty_scale": args.near_target_action_penalty_scale,
        "near_target_action_penalty_radius": args.near_target_action_penalty_radius,
        "smoothness_penalty_scale": args.smoothness_penalty_scale,
        "joint_limit_penalty_scale": args.joint_limit_penalty_scale,
        "success_bonus": args.success_bonus,
        "target_position": args.target_position,
        "target_low": args.target_low,
        "target_high": args.target_high,
        "hard_target_position": args.hard_target_position,
        "hard_target_positions": args.hard_target_positions,
        "hard_target_ratio": args.hard_target_ratio,
        "hard_target_noise": args.hard_target_noise,
        "reset_noise": args.reset_noise,
        "reset_world_on_reset": args.reset_world_on_reset,
    }


def _make_env(env_kwargs):
    def _init():
        env = CArmA3GazeboReachingEnv(**env_kwargs)
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
    parser.add_argument(
        "--near-target-action-scale-radius",
        type=float,
        default=0.0,
        help="Enable distance-based action scaling inside this TCP distance. 0 disables scaling.",
    )
    parser.add_argument("--near-target-action-scale-min", type=float, default=0.25)
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
    parser.add_argument(
        "--near-target-action-penalty-scale",
        type=float,
        default=0.0,
        help="Extra action penalty when TCP is within the near-target radius.",
    )
    parser.add_argument("--near-target-action-penalty-radius", type=float, default=0.08)
    parser.add_argument("--smoothness-penalty-scale", type=float, default=0.0)
    parser.add_argument("--joint-limit-penalty-scale", type=float, default=0.0)
    parser.add_argument("--success-bonus", type=float, default=0.0)
    parser.add_argument("--target-position", type=parse_target_position, default=None, help="Fixed target as x,y,z.")
    parser.add_argument("--target-low", type=parse_target_position, default=None, help="Target sampling lower bound as x,y,z.")
    parser.add_argument("--target-high", type=parse_target_position, default=None, help="Target sampling upper bound as x,y,z.")
    parser.add_argument("--hard-target-position", type=parse_target_position, default=None, help="Replay target center as x,y,z.")
    parser.add_argument(
        "--hard-target-positions",
        type=parse_target_positions,
        default=None,
        help="Replay target centers as 'x,y,z;x,y,z'. Overrides --hard-target-position when replay is sampled.",
    )
    parser.add_argument("--hard-target-ratio", type=float, default=0.0)
    parser.add_argument("--hard-target-noise", type=float, default=0.03)
    parser.add_argument("--reset-noise", type=float, default=0.0, help="Uniform joint reset noise in radians.")
    parser.add_argument("--reset-world-on-reset", action="store_true", help="Call /reset_world before each env reset.")
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
    parser.add_argument("--clip-range", type=float, default=0.2, help="PPO clipping range.")
    parser.add_argument("--ent-coef", type=float, default=0.0, help="Entropy coefficient.")
    parser.add_argument("--vf-coef", type=float, default=0.5, help="Value loss coefficient.")
    parser.add_argument("--gae-lambda", type=float, default=0.95, help="PPO GAE lambda.")
    parser.add_argument("--run-name", default=None, help="Optional model filename suffix to avoid overwriting runs.")
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

    env = DummyVecEnv([_make_env(env_kwargs)])
    algo_cls = ALGORITHMS[args.algo]
    model_kwargs = {
        "learning_rate": args.learning_rate,
        "n_steps": args.n_steps,
        "ent_coef": args.ent_coef,
        "vf_coef": args.vf_coef,
    }
    if args.algo == "ppo":
        model_kwargs["batch_size"] = args.batch_size
        model_kwargs["clip_range"] = args.clip_range
        model_kwargs["gae_lambda"] = args.gae_lambda

    print(f"algo={args.algo}")
    print("num_envs=1")
    print("vec_env=dummy")
    print(f"max_steps={args.max_steps}")
    print(f"success_threshold={args.success_threshold}")
    print(f"timesteps={args.timesteps}")
    print(f"rollout_batch={args.n_steps}")
    print(f"action_scale={args.action_scale}")
    print(f"near_target_action_scale_radius={args.near_target_action_scale_radius}")
    print(f"near_target_action_scale_min={args.near_target_action_scale_min}")
    print(f"command_duration={args.command_duration}")
    print(f"command_settle_time={args.command_settle_time}")
    print(f"command_timeout={args.command_timeout}")
    print(f"joint_target_tolerance={args.joint_target_tolerance}")
    print(f"distance_reward_scale={args.distance_reward_scale}")
    print(f"progress_reward_scale={args.progress_reward_scale}")
    print(f"distance_regression_penalty_scale={args.distance_regression_penalty_scale}")
    print(f"action_penalty_scale={args.action_penalty_scale}")
    print(f"near_target_action_penalty_scale={args.near_target_action_penalty_scale}")
    print(f"near_target_action_penalty_radius={args.near_target_action_penalty_radius}")
    print(f"smoothness_penalty_scale={args.smoothness_penalty_scale}")
    print(f"joint_limit_penalty_scale={args.joint_limit_penalty_scale}")
    print(f"success_bonus={args.success_bonus}")
    print(f"target_position={args.target_position}")
    print(f"target_low={args.target_low}")
    print(f"target_high={args.target_high}")
    print(f"hard_target_position={args.hard_target_position}")
    print(f"hard_target_positions={args.hard_target_positions}")
    print(f"hard_target_ratio={args.hard_target_ratio}")
    print(f"hard_target_noise={args.hard_target_noise}")
    print(f"reset_noise={args.reset_noise}")
    print(f"reset_world_on_reset={args.reset_world_on_reset}")
    print(f"learning_rate={args.learning_rate}")
    print(f"batch_size={args.batch_size}")
    print(f"clip_range={args.clip_range}")
    print(f"ent_coef={args.ent_coef}")
    print(f"vf_coef={args.vf_coef}")
    print(f"gae_lambda={args.gae_lambda}")

    if args.load_model:
        model = algo_cls.load(
            args.load_model,
            env=env,
            device=args.device,
            seed=args.seed,
            verbose=1,
            custom_objects=model_kwargs,
        )
        print(f"loaded_model={args.load_model}")
        print("loaded_model_custom_objects=enabled")
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

    if args.run_name:
        model_suffix = args.run_name
    else:
        model_suffix = f"{args.timesteps}_more_steps" if args.load_model else f"{args.timesteps}_steps"
    model_path = save_dir / f"{args.algo}_gazebo_reaching_{model_suffix}"
    model.save(model_path)
    print(f"saved_model={model_path}.zip")

    print(f"eval_episodes={args.eval_episodes}")
    if args.eval_episodes > 0:
        distances, rewards = _evaluate(model, args.eval_episodes, env_kwargs)
        mean_distance = sum(distances) / len(distances)
        mean_reward = sum(rewards) / len(rewards)
        print(f"eval_mean_distance={mean_distance:.4f}")
        print(f"eval_mean_reward={mean_reward:.4f}")

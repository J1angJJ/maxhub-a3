import argparse
from pathlib import Path

from stable_baselines3 import A2C, PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor

from carm_rl_env.reaching_env import CArmA3ReachingEnv


ALGORITHMS = {
    "ppo": PPO,
    "a2c": A2C,
}


def _make_env(max_steps):
    return Monitor(CArmA3ReachingEnv(max_steps=max_steps))


def _evaluate(model, episodes, max_steps):
    distances = []
    rewards = []
    for episode in range(episodes):
        env = CArmA3ReachingEnv(max_steps=max_steps)
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
    parser.add_argument("--num-envs", type=int, default=1, help="Number of parallel vectorized env instances.")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="auto", help="auto, cpu, cuda, or cuda:0")
    parser.add_argument("--save-dir", default="/workspace/rl_ws/artifacts/reaching")
    parser.add_argument("--eval-episodes", type=int, default=5)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--n-steps", type=int, default=256, help="PPO rollout steps, or A2C update steps.")
    parser.add_argument("--batch-size", type=int, default=64, help="PPO minibatch size.")
    parser.add_argument("--check-env", action="store_true")
    args = parser.parse_args()

    save_dir = Path(args.save_dir)
    save_dir.mkdir(parents=True, exist_ok=True)

    raw_env = CArmA3ReachingEnv(max_steps=args.max_steps)
    if args.check_env:
        check_env(raw_env, warn=True)

    env = make_vec_env(
        CArmA3ReachingEnv,
        n_envs=args.num_envs,
        seed=args.seed,
        monitor_dir=str(save_dir / "monitor"),
        env_kwargs={"max_steps": args.max_steps},
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
    print(f"timesteps={args.timesteps}")
    print(f"rollout_batch={args.n_steps * args.num_envs}")

    model = algo_cls(
        "MlpPolicy",
        env,
        verbose=1,
        seed=args.seed,
        device=args.device,
        **model_kwargs,
    )
    model.learn(total_timesteps=args.timesteps)

    model_path = save_dir / f"{args.algo}_reaching_{args.timesteps}_steps"
    model.save(model_path)
    print(f"saved_model={model_path}.zip")

    distances, rewards = _evaluate(model, args.eval_episodes, args.max_steps)
    mean_distance = sum(distances) / len(distances)
    mean_reward = sum(rewards) / len(rewards)
    print(f"eval_episodes={args.eval_episodes}")
    print(f"eval_mean_distance={mean_distance:.4f}")
    print(f"eval_mean_reward={mean_reward:.4f}")

from carm_rl_env.reaching_env import CArmA3ReachingEnv


def main():
    env = CArmA3ReachingEnv()
    obs, info = env.reset(seed=7)
    print(f"reset obs_shape={obs.shape} distance={info['distance']:.4f}")

    total_reward = 0.0
    for step in range(20):
        obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
        total_reward += reward
        print(
            f"step={step + 1:02d} reward={reward:.4f} "
            f"distance={info['distance']:.4f} terminated={terminated} truncated={truncated}"
        )
        if terminated or truncated:
            break

    print(f"total_reward={total_reward:.4f}")

import gymnasium as gym
env = gym.make("CarRacing-v2", domain_randomize=True)

observation, info = env.reset(seed=42)
for _ in range(1000):
    action = env.action_space.sample()
    observation, reward, terminated, truncated, info = env.step(action)

    print(observation, reward, terminated, truncated, info)

    if terminated or truncated:
        observation, info = env.reset()
env.close()

import gymnasium as gym
from stable_baselines3 import PPO
import os


# First, we create our environment called LunarLander-v2
env = gym.make("LunarLander-v2", render_mode="human")
env.reset()

model_name = "2990000.zip"
models_dir = "models\PPO-1688407368"

model_path = f"{models_dir}/{model_name}"

model = PPO.load(model_path, env=env)


episodes = 10
for i in range(episodes):
    obs, _ = env.reset()
    done = False
    while not done:
        action, _ = model.predict(observation=obs, deterministic=True)
        obs, reward, done, truncated, info = env.step(action)

env.close()

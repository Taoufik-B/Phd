import gymnasium as gym


from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor

import os
import time

models_dir = f"models/PPO-{int(time.time())}"
logs_dir = f"logs/PPO-{int(time.time())}"

if not os.path.exists(models_dir):
    os.makedirs(models_dir)

if not os.path.exists(logs_dir):
    os.makedirs(logs_dir)


# First, we create our environment called LunarLander-v2
env = gym.make("LunarLander-v2")
env.reset()
# env = make_vec_env('LunarLander-v2', n_envs=16)


# We added some parameters to accelerate the training
model = PPO(
    policy='MlpPolicy',
    env=env,
    n_steps=1024,
    batch_size=64,
    n_epochs=4,
    gamma=0.999,
    gae_lambda=0.98,
    ent_coef=0.01,
    verbose=1,
    tensorboard_log=logs_dir)


TIMESTEPS = 10_000
for i in range(1, 100):
    model.learn(total_timesteps=TIMESTEPS,
                reset_num_timesteps=False, tb_log_name="PPO")
    # Save the model
    model.save(f"{models_dir}/{TIMESTEPS*i}")

env.close()

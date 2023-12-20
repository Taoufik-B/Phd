import gymnasium as gym


from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.monitor import Monitor

import os
import time

# First, we create our environment called LunarLander-v2
env = gym.make("LunarLander-v2")
env.reset()

model_name = "1990000.zip"
models_dir = "models\PPO-1688407368"

model_path = f"{models_dir}/{model_name}"


model = PPO.load(model_path, env=env)

FROM = 200
TO = 300

TIMESTEPS = 10_000
for i in range(FROM, TO):
    model.learn(total_timesteps=TIMESTEPS,
                reset_num_timesteps=False, tb_log_name="PPO")
    # Save the model
    model.save(f"{models_dir}/{TIMESTEPS*i}")

env.close()

import gym

from warnings import filterwarnings
filterwarnings(action='ignore', category=DeprecationWarning,
               message='`np.bool8` is a deprecated alias for `np.bool_`')

env = gym.make("MountainCar-v0", render_mode="human")
observation, info = env.reset()

done = False


for _ in range(100):
    action = 2
    observation, reward, terminated, truncated, info = env.step(action)
    print(observation, reward)
    print(env.observation_space)
    print(env.action_space)
    if terminated or truncated:
        print(f"Env reset at iteration {_} with {observation, reward}")
        observation, info = env.reset()


print("Done")
env.close()

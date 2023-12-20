import gym
import sys
import numpy as np

from warnings import filterwarnings
filterwarnings(action='ignore', category=DeprecationWarning,
               message='`np.bool8` is a deprecated alias for `np.bool_`')

env = gym.make("MountainCar-v0")


# Constants
DISCRETE_OS_SIZE = [20, 20]
DISCRETE_OS_WIN_SIZE = (env.observation_space.high -
                        env.observation_space.low)/DISCRETE_OS_SIZE

# RENDER_EVERY = 3000

# # Q Learning settings
# LEARNING_RATE = 0.1
# DISCOUNT = 0.95
# EPISODES = 25_000

# # Exploration settings
# epsilon = 1  # not a constant, qoing to be decayed
# START_EPSILON_DECAYING = 1
# END_EPSILON_DECAYING = EPISODES//2
# epsilon_decay_value = epsilon/(END_EPSILON_DECAYING - START_EPSILON_DECAYING)

# Helper functions


def get_discrete_state(state):
    discrete_state = (state-env.observation_space.low)/DISCRETE_OS_WIN_SIZE
    return tuple(discrete_state.astype(np.int_))


# q_table_ref = np.random.uniform(
#     low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n]))

q_table = np.load('trained_MountainCar.npy')

print(q_table.shape)
# print(q_table_ref.shape)

for episode in range(10):
    initial_observation = env.reset()
    discrete_state = get_discrete_state(initial_observation)
    done = False
    exit_loop = False
    print(episode)
    i = 0
    while not done:
        try:
            # Get action from Q table
            action = np.argmax(q_table[discrete_state])
            observation, reward, done, _ = env.step(action)
            discrete_observation = get_discrete_state(observation)
            env.render(mode='human')
            discrete_state = discrete_observation

        except KeyboardInterrupt or SystemExit:
            print("Ctr-C Keyboard Exit")
            exit_loop = True
            break
    print(f"{i}---goal Reached")

    if exit_loop:
        break


if env is not None:
    env.close()
    print("Closing Environment")

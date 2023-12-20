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

RENDER_EVERY = 3000

# Q Learning settings
LEARNING_RATE = 0.1
DISCOUNT = 0.95
EPISODES = 25_000

# Exploration settings
epsilon = 1  # not a constant, qoing to be decayed
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES//2
epsilon_decay_value = epsilon/(END_EPSILON_DECAYING - START_EPSILON_DECAYING)

# Helper functions


def get_discrete_state(state):
    discrete_state = (state-env.observation_space.low)/DISCRETE_OS_WIN_SIZE
    return tuple(discrete_state.astype(np.int_))


q_table = np.random.uniform(
    low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n]))

print(q_table.shape)

# init discrete state
for episode in range(EPISODES):
    initial_observation = env.reset()
    discrete_state = get_discrete_state(initial_observation)
    done = False
    exit_loop = False
    if episode % RENDER_EVERY == 0:
        render = True
        print(episode)
    else:
        render = False
    while not done:
        try:
            if np.random.random() > epsilon:
                # Get action from Q table
                action = np.argmax(q_table[discrete_state])
            else:
                # Get random action
                action = np.random.randint(0, env.action_space.n)
            observation, reward, done, _ = env.step(action)
            discrete_observation = get_discrete_state(observation)
            if episode % RENDER_EVERY == 0:
                env.render(mode='human')

            if not done:
                # Maximum possible Q value in next step ( for new observation)
                max_future_q = np.max(q_table[discrete_observation])
                # Current Q value (for current state and performed action)
                current_q = q_table[discrete_state + (action, )]
                # Equation for the new Q value
                new_q = (1-LEARNING_RATE)*current_q + LEARNING_RATE * \
                    (reward + DISCOUNT*max_future_q)
                # Update the Q table with new Q value
                q_table[discrete_state + (action, )] = new_q
            elif observation[0] >= env.goal_position:
                q_table[discrete_state + (action, )] = 0

            discrete_state = discrete_observation

        except KeyboardInterrupt or SystemExit:
            print("Ctr-C Keyboard Exit")
            exit_loop = True
            break

    if exit_loop:
        break
    # Decaying is being done every episode if episode number is within decaying range
    if END_EPSILON_DECAYING >= episode >= START_EPSILON_DECAYING:
        epsilon -= epsilon_decay_value


# q_table.tofile('trained_MountainCar.csv', sep=',', format='%d')
np.save('trained_MountainCar.npy', q_table)

if env is not None:
    env.close()
    print("Closing Environment")

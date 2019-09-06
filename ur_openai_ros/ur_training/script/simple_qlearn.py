#!/usr/bin/env python

'''
    Original Training code made by Machine Learning with Phil https://youtu.be/ViwBAK8Hd7Q
    Just transcribed and moded for our scenario by Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import rospy
import gym
import numpy as np
import matplotlib.pyplot as plt
import ur_env

MAX_STATES = 10**4
GAMMA = 0.9
ALPHA = 0.01

# Create the Gym environment
env = gym.make('UR-v0')
print "ENV Loaded..."

def max_dict(d):
    max_v = float('-inf')
    for key, val in d.items():
        if val > max_v:
            max_v = val
            max_key = key
    return max_key, max_v


def create_bins():
    """
    obs[0]  ->  cart position   ... -4.8     4.8
    obs[1]  ->  cart velocity   ... -inf     inf
    obs[2]  ->  pole angle      ... -41.8     41.8
    obs[3]  ->  pole velocity   ... -inf     inf
    :return:
    """

    bins = np.zeros((4,10))
    bins[0] = np.linspace(-4.8,4.8,10)
    bins[1] = np.linspace(-5, 5, 10)
    bins[2] = np.linspace(-0.418, 0.418, 10)
    bins[3] = np.linspace(-5, 5, 10)

    return bins


def assign_bins(observation, bins):
    state = np.zeros(4)
    for i in range(4):
        state[i] = np.digitize(observation[i], bins[i])
    return state


def get_state_as_string(state):
    string_state = ''.join(str(int(e)) for e in state)
    return string_state


def get_all_states_as_string():
    states = []
    for i in range(MAX_STATES):
        states.append(str(i).zfill(4))
    return states


def initialise_Q():
    Q = {}

    all_states = get_all_states_as_string()
    for state in all_states:
        Q[state] = {}
        for action in range(env.action_space.n):
            Q[state][action] = 0
    return Q


def play_one_game(bins, Q, eps=0.5):
    observation = env.reset()
    done = False
    cnt = 0
    state = get_state_as_string(assign_bins(observation,bins))
    total_reward = 0

    while not done:
        cnt += 1

        if np.random.uniform() < eps:
            act = env.action_space.sample() # epsilon greedy
        else:
            act = max_dict(Q[state][0])

        observation, reward, done, _ = env.step(act)

        total_reward += reward

        if done and cnt < 200:
            reward = -300

        state_new = get_state_as_string(assign_bins(observation,bins))

        a1, max_q_a1s1 = max_dict(Q[state_new])
        Q[state][act] += ALPHA*(reward + GAMMA*max_q_a1s1 - Q[state][act])
        state, act = state_new, a1

    return total_reward, cnt


def play_many_games(bins, N=10000):
    Q = initialise_Q()

    length = []
    reward = []

    for n in range(N):
        eps = 1.0 / np.sqrt(n+1)

        episode_reward, episode_length = play_one_game(bins, Q, eps)

        if n % 100 == 0:
            rospy.loginfo("n="+str(n)+",eps="+str(eps)+", episode_reward="+str(episode_reward))
        length.append(episode_length)
        reward.append(episode_reward)
    return length, reward


def plot_running_avg(totalrewards):
    N = len(totalrewards)
    running_avg = np.empty(N)
    for t in range(N):
        running_avg[t] = np.mean(totalrewards[max(0,t-100):(t+1)])
    plt.plot(running_avg)
    plt.title("Running Average")
    plt.show()


if __name__ == '__main__':

    rospy.init_node('simple_ur_gym', anonymous=True, log_level=rospy.WARN)
    bins = create_bins()
    episode_lengths, episode_rewards = play_many_games(bins)

    plot_running_avg(episode_rewards)





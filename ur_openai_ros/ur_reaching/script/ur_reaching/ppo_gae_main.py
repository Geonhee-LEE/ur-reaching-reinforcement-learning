from collections import deque
import numpy as np
import tensorflow as tf
from matplotlib import pyplot as plt
from sklearn.utils import shuffle
from ur_reaching.algorithm.ppo_gae import PPOGAEAgent

# import our training environment
import gym
from ur_reaching.env.ur_reaching_env import URSimReaching

import rospy
import rospkg

seed = 0
obs_dim = 15 # env.observation_space.shape[0]
n_act = 6 #config: act_dim #env.action_space.n
agent = PPOGAEAgent(obs_dim, n_act, epochs=10, hdim=16, policy_lr=3e-3, value_lr=1e-3, max_std=1.0,
                            clip_range=0.2, seed=seed)


'''
PPO Agent with Gaussian policy
'''
def run_episode(env, animate=False): # Run policy and collect (state, action, reward) pairs
    obs = env.reset()
    observes, actions, rewards, infos = [], [], [], []
    done = False
    while not done:
        obs = np.array(obs)
        obs = obs.astype(np.float32).reshape((1, -1))
        observes.append(obs)
        
        action = agent.get_action(obs)
        actions.append(action)
        obs, reward, done, info = env.step(action)
        
        if not isinstance(reward, float):
            reward = np.asscalar(reward)
        rewards.append(reward)
        infos.append(info)
        
    return (np.concatenate(observes), np.array(actions), np.array(rewards, dtype=np.float32), infos)

def run_policy(env, episodes): # collect trajectories
    total_steps = 0
    trajectories = []
    for e in range(episodes):
        observes, actions, rewards, infos = run_episode(env)
        total_steps += observes.shape[0]
        trajectory = {'observes': observes,
                      'actions': actions,
                      'rewards': rewards,
                      'infos': infos}
        trajectories.append(trajectory)
    return trajectories
        
def add_value(trajectories, val_func): # Add value estimation for each trajectories
    for trajectory in trajectories:
        observes = trajectory['observes']
        values = val_func.get_value(observes)
        trajectory['values'] = values
        print ("observes: ", observes)
        print ("values): ",values, "type: ", type(values), "size: ", values.shape)

def add_gae(trajectories, gamma=0.99, lam=0.98): # generalized advantage estimation (for training stability)
    for trajectory in trajectories:
        rewards = trajectory['rewards']
        values = trajectory['values']
        
        # temporal differences
        print ("rewards: ", rewards)
        print ("values): ",values, "type: ", type(values), "size: ", values.shape)
        print ("np.append(values[1:],0): ", np.append(values,0))
        tds = rewards + np.append(values, 0) * gamma - values
        advantages = np.zeros_like(tds)
        advantage = 0
        for t in reversed(range(len(tds))):
            advantage = tds[t] + lam*gamma*advantage
            advantages[t] = advantage
        trajectory['advantages'] = advantages

def add_rets(trajectories, gamma=0.99): # compute the returns
    for trajectory in trajectories:
        rewards = trajectory['rewards']
        
        returns = np.zeros_like(rewards)
        ret = 0
        for t in reversed(range(len(rewards))):
            ret = rewards[t] + gamma*ret
            returns[t] = ret            
        trajectory['returns'] = returns

def build_train_set(trajectories):
    observes = np.concatenate([t['observes'] for t in trajectories])
    actions = np.concatenate([t['actions'] for t in trajectories])
    returns = np.concatenate([t['returns'] for t in trajectories])
    advantages = np.concatenate([t['advantages'] for t in trajectories])

    # Normalization of advantages 
    # In baselines, which is a github repo including implementation of PPO coded by OpenAI, 
    # all policy gradient methods use advantage normalization trick as belows.
    # The insight under this trick is that it tries to move policy parameter towards locally maximum point.
    # Sometimes, this trick doesnot work.
    advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-6)

    return observes, actions, advantages, returns

    
def main():
    # Can check log msgs according to log_level {rospy.DEBUG, rospy.INFO, rospy.WARN, rospy.ERROR} 
    rospy.init_node('ur_gym', anonymous=True, log_level=rospy.DEBUG)
    
    env = gym.make('URSimReaching-v0')
    env._max_episode_steps = 10000
    np.random.seed(seed)
    tf.set_random_seed(seed)
    env.seed(seed=seed)

    avg_return_list = deque(maxlen=10)
    avg_pol_loss_list = deque(maxlen=10)
    avg_val_loss_list = deque(maxlen=10)

    episode_size = 1
    batch_size = 16
    nupdates = 10000 

    # save fig
    x_data = []
    y_data = []
    axes = plt.gca()
    axes.set_xlim(0, 350)
    axes.set_ylim(-350, +1)
    line, = axes.plot(x_data, y_data, 'r-')

    for update in range(nupdates+1):
        trajectories = run_policy(env, episodes=episode_size)
        add_value(trajectories, agent)
        add_gae(trajectories)
        add_rets(trajectories)
        observes, actions, advantages, returns = build_train_set(trajectories)

        print ("advantages: ", advantages.size, type(advantages))
        print ("returns: ", returns.size, type(returns))
        print ("actions: ", actions.size, type(actions))
        pol_loss, val_loss, kl, entropy = agent.update(observes, actions, advantages, returns, batch_size=batch_size)

        avg_pol_loss_list.append(pol_loss)
        avg_val_loss_list.append(val_loss)
        
        avg_return_list.append([np.sum(t['rewards']) for t in trajectories])
        if (update%1) == 0:
            print('[{}/{}] return : {:.3f}, value loss : {:.3f}, policy loss : {:.3f}, policy kl : {:.5f}, policy entropy : {:.3f}'.format(
                update, nupdates, np.mean(avg_return_list), np.mean(avg_val_loss_list), np.mean(avg_pol_loss_list), kl, entropy))
            
        x_data.append(update)
        y_data.append(np.mean(avg_return_list))
        
        if (update%5) == 0:
            line.set_xdata(x_data)
            line.set_ydata(y_data)
            plt.draw()  
            plt.pause(1e-17)
            plt.savefig("./results/ppo_with_gae_avg_return_list.png")

        if (np.mean(avg_return_list) > 1): # Threshold return to success 
            print('[{}/{}] return : {:.3f}, value loss : {:.3f}, policy loss : {:.3f}'.format(update,nupdates, np.mean(avg_return_list), np.mean(avg_val_loss_list), np.mean(avg_pol_loss_list)))
            print('The problem is solved with {} episodes'.format(update*episode_size))
            break

	env.close()

if __name__ == '__main__':
    main()
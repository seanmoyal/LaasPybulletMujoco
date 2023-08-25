from gym_examples.gym_examples.envs.AlbertEnv import AlbertEnv



#env = gym.make('gym_examples/Albert-v0')
env = AlbertEnv()
env.reset()



episodes=10

for ep in range(episodes):
    env.reset()
    done=False
    while not done:
        env.render()
        current_obs, reward, done, info = env.step(env.action_space.sample())



env.close()
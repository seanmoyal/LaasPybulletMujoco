
from stable_baselines3 import PPO
from gym_albert_mujoco.gym_examples.envs.AlbertEnv import AlbertEnv

print("--------------Loading Model--------------")
env = AlbertEnv()
loaded_model = PPO.load("trained_model_directory/ppo_model")
print("--------------Start Simulation--------------")
episodes = 10
for ep in range(episodes):
    obs = env.reset()
    done=False
    while not done:
        env.render()
        action, _ = loaded_model.predict(obs)
        obs, reward, done, info = env.step(action)



# Close the environment
env.close()

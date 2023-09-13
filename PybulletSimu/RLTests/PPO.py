import numpy as np
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

from gym_albert_pybullet.gym_examples.envs.AlbertEnv import AlbertEnv
# Define a custom features extractor

# Custom features extractor
class CustomFeaturesExtractor(BaseFeaturesExtractor):
    def __init__(self, observation_space, features_dim):
        super(CustomFeaturesExtractor, self).__init__(observation_space, features_dim)
        self.fc = nn.Linear(np.prod(observation_space.shape), features_dim)

    def forward(self, observations):
        x = torch.relu(self.fc(observations))
        return x

# Create the custom MLP network
class MLP(nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim):
        super(MLP, self).__init__()
        self.fc1 = nn.Linear(input_dim, hidden_dim)
        self.fc2 = nn.Linear(hidden_dim, hidden_dim)
        self.fc3 = nn.Linear(hidden_dim, output_dim)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = self.fc3(x)
        return x


# Define the environment
# Create and wrap the environment

print("--------------Init Environment--------------")
env = AlbertEnv()
#env = VecNormalize(env, norm_obs=True, norm_reward=False)

# Define the network dimensions
input_dim = np.prod(env.observation_space.shape)
hidden_dim = 128
#output_dim = env.action_space.shape
output_dim=3
# Create the custom MLP policy
policy_kwargs = dict(
    features_extractor_class=CustomFeaturesExtractor,
    features_extractor_kwargs=dict(features_dim=hidden_dim),
    net_arch=[dict(pi=[hidden_dim], vf=[hidden_dim])]
)

# Instantiate the PPO agent with the custom MLP policy
print("--------------Init Model--------------")
model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1)

# Train the PPO model
print("--------------Start Learning--------------")
model.learn(total_timesteps=10000)
print("--------------End Learning--------------")
# Save the trained model
print("--------------Saving trained Model--------------")
model.save("ppo_model")
print("--------------Trained Model Saved--------------")
# Load the saved model
print("--------------Loading Model--------------")

loaded_model = PPO.load("./ppo_model")
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

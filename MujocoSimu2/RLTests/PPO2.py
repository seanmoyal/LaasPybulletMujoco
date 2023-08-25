import gym
import numpy as np
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from gym_examples3.gym_examples.envs.AlbertEnv import AlbertEnv
from stable_baselines3.common.logger import configure
from CustomCallback import CustomCallback
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
        x = torch.sigmoid(self.fc3(x))
        return x


# Define the environment
# Create and wrap the environment

print("--------------Init Environment--------------")
env = AlbertEnv()
#env = VecNormalize(env, norm_obs=True, norm_reward=False)

# Create the vectorized environment
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
model = PPO("MlpPolicy", env, policy_kwargs=policy_kwargs, verbose=1,tensorboard_log="./ppo_albert_tensorboard/")

# Train the PPO model
print("--------------Start Learning--------------")
model.learn(total_timesteps=15000000,tb_log_name="albert_training",callback=CustomCallback())
print("--------------End Learning--------------")
# Save the trained model
print("--------------Saving trained Model--------------")
model.save("ppo_model")
print("--------------Trained Model Saved--------------")

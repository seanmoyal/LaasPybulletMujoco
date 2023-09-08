from gym.envs.registration import register

register(
    id='gym_albert_pybullet/Albert-v1',
    entry_point='gym_albert_pybullet.envs:AlbertEnv',
    max_episode_steps=2400,
)
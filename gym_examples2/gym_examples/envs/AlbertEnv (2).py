import os.path

import gym
import mujoco.viewer
import numpy as np
import mujoco as mj
from gym.spaces import Box, MultiDiscrete, Discrete, Dict, MultiBinary
from numpy.random import default_rng
import time
from MujocoSimu.ObjetsEnvironnement.AlbertCube import AlbertCube
from MujocoSimu.ObjetsEnvironnement.Room import Room
from MujocoSimu.ObjetsEnvironnement.RoomManager import RoomManager
from XmlConversionDirectory.xmlMerger import merge_mjcf_files


class AlbertEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, character=1):

        self.room_manager = RoomManager()
        project_name="testEnviSim"
        project_path = get_absolute_path_project(project_name).replace('\\', '/')
        xml_directory_path=project_path+"/xmlDirectory/"
        room_manager_path=xml_directory_path+"Room2bis.xml"
        room_manager_xml = room_manager_path
        albert_xml = xml_directory_path+"Actor.xml"


        merge_mjcf_files(room_manager_xml, albert_xml, "AlbertEnvironment2")

        albert_environnement = xml_directory_path+"AlbertEnvironment2.xml"
        # initialisation mujoco
        self.model = mj.MjModel.from_xml_path(albert_environnement)
        self.data = mj.MjData(self.model)
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.room_manager.add_room(Room(self.model, name='room1'))
        self.character = AlbertCube(room_manager=self.room_manager, data=self.data, model=self.model)

        self.state_space = Dict(
            {
                "CharacterPosition": Box(low=np.array([0., 0., -5.]), high=np.array([10., 10., 10.])),
                "doorState": Discrete(2),
                "doorPosition": MultiDiscrete(np.array([3, 10])),  # a voir
                "buttonsState": MultiBinary(3),  # ici 3 bouttons
                "contactPoints": MultiDiscrete(np.array([6, 6, 6, 6, 6, 6]))
            }
        )
        self.curr_state = self.character.current_state
        self.prev_state = self.character.get_previous_state()

        # Observation_space : 21 * tuple(objType,distance) for each Ray
        self.observation_space = Box(
            low=np.concatenate((np.array([0 for _ in range(105)]), np.array([0 for _ in range(105)]))),
            high=np.concatenate([np.array([5 for _ in range(105)]), np.array([10 for _ in range(105)])]), shape=(210,))

        # Action_space : [rotate,move,jump]
        self.action_space = MultiDiscrete(np.array([3, 3, 2]))
        # [[leftRot,rightRot],[MoveBackwards,NotMove,MoveForward],[DontJump,Jump]]

        # RNG :
        self.rng = default_rng()

        self.current_obs = None
        self.previous_obs = None


        self.time_episode = 100  # in seconds
        self.time_passed = 0

        self.step_count=0
        pass

    def step(self, action):
        # given current obs and action returns the next observation, the reward, done and optionally additional info
        self.character.take_action(action)
        # compute next obs
        self.step_count+=1
        if self.step_count==48:

            self.current_obs = self.character.get_observation(self.viewer)
            self.step_count=0
        self.update_state()

        # compute reward
        reward = 0
        contact = self.curr_state["contactPoints"]
        if action[1]==0:
            reward-=0.05
        if action[2] == 1:
            reward -= 0.05
        if (3 in contact or 4 in contact or 5 in contact):
            reward -= 0.1
        if (self.achieved_maze()):
            reward += 2
        if (self.button_distance() != 0):
            reward += 1+(1-self.time_passed/self.time_episode)
        if self.curr_state["CharacterPosition"][2] <= self.room_manager.room_array[0].global_coord[
            2]:  # a changer pr que ce soit qu'une fois ( quand il tombe )
            reward -= 0.5
        # compute done

        self.time_passed += 1 / 240  # 1/240 s is the time step
        done = False
        if (self.time_passed >= self.time_episode or self.character.has_fallen() or self.achieved_maze()):
            done = True
        info={}
        char_pos = self.curr_state["CharacterPosition"]
        door_pos = self.curr_state["doorPosition"]
        dist = np.sqrt(sum([(char_pos[i] - door_pos[i]) ** (2) for i in range(2)]))
        info["distance"]=dist

        return self.current_obs, reward, done, info

    def reset(self):
        # Initial position : in front of the first window
        # these paremeters will need a change following the change of the room's or albert's characteristics
        # since these positions are those of the center of mass
        room = self.character.room_manager.room_array[self.character.actual_room]
        room.reset_room(self.model, self.character)

        x_alb = self.rng.uniform(1, 5)
        y_alb = self.rng.uniform(1, 3)
        z_alb = 0.75

        ori_euler = [0, 0, self.rng.uniform(-np.pi, np.pi)]

        self.character.reset_position_orientation([x_alb, y_alb, z_alb], ori_euler)


        self.current_obs = self.character.get_observation(self.viewer)


        self.time_passed = 0

        return self.current_obs

    def render(self):

        mj.mj_step(self.model, self.data)
        self.viewer.sync()
        #time.sleep(1 / 240)

    def close(self):
        self.viewer.close()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def button_distance(self):
        n = len(self.character.current_state["buttonsState"])
        if self.prev_state == None:
            return 0

        d = sum([np.abs(self.curr_state["buttonsState"][i] - self.prev_state["buttonsState"][i]) for i in range(n)])
        return d

    def achieved_maze(self):
        char_pos = self.curr_state["CharacterPosition"]
        door_pos = self.curr_state["doorPosition"]
        dist = np.sqrt(sum([(char_pos[i] - door_pos[i]) ** (2) for i in range(2)]))
        return (dist <= 0.3)

    def update_state(self):
        self.curr_state = self.character.current_state
        self.prev_state = self.character.get_previous_state()


def get_absolute_path_project(project_name):

    script_directory = os.path.dirname(os.path.abspath(__file__))

    current_directory = script_directory
    while current_directory != os.path.dirname(current_directory):
        if os.path.basename(current_directory) == project_name:
            return current_directory
        current_directory = os.path.dirname(current_directory)

    # If the specified directory is not found, return None
    return None
import gym
import numpy as np
import pybullet as p
import pybullet_data
from gym.spaces import Box, MultiDiscrete, Tuple, Discrete, Dict, MultiBinary
from numpy.random import default_rng
import time
from PybulletSimu.ObjetsEnvironnement.Cube import Cube
from PybulletSimu.ObjetsEnvironnement.Button import Button
from PybulletSimu.ObjetsEnvironnement.Door import Door
from PybulletSimu.ObjetsEnvironnement.AlbertCube import AlbertCube
from PybulletSimu.ObjetsEnvironnement.Room import Room
from PybulletSimu.ObjetsEnvironnement.RoomManager import RoomManager


class AlbertEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, character=1):

        # initialisation
        physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -100)
        plane_id = p.loadURDF("plane.urdf")

        self.room_manager = RoomManager()
        # Creating and Adding a room
        base_cube = Cube()
        button = Button()
        door = Door()
        self.room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
        self.room_manager.room_array[0].build_basic_room(door)
        self.room_manager.room_array[0].build_button(button, 3, 4, 0)
        self.character = AlbertCube(self.room_manager)

        linear_friction = 4
        angular_friction = 4
        p.changeDynamics(character.id, -1, linearDamping=linear_friction, angularDamping=angular_friction)

        self.state_space = Dict(
            {
                "CharacterPosition": Box(low=np.array([0., 0., -5.]), high=np.array([10., 10., 10.])),
                "doorState": Discrete(2),
                "doorPosition": MultiDiscrete(np.array([3, 10])),
                "buttonsState": MultiBinary(3),# considering 3 buttons
                "contactPoints": MultiDiscrete(np.array([6, 6, 6, 6, 6, 6]))
            }
        )
        self.curr_state = self.character.current_state
        self.prev_state = self.character.get_previous_state()

        # Observation_space : 21 * tuple(objType,distance) for each ray
        self.observation_space = Box(
            low=np.concatenate((np.array([0 for _ in range(105)]), np.array([0 for _ in range(105)]))),
            high=np.concatenate([np.array([5 for _ in range(105)]), np.array([10 for _ in range(105)])]), shape=(210,))

        # Action_space : [rotate,move,jump] moving and jumping only possible if albert is on the ground
        self.action_space = MultiDiscrete(np.array([3, 3, 2]))
        # [[LeftRotation,RightRotation],[MoveBackwards,NotMove,MoveForward],[NotJump,Jump]]

        # RNG :
        self.rng = default_rng()

        # current observation
        self.current_obs = None
        self.previous_obs = None

        # done
        self.time_episode = 10  # 10 secs
        self.time_passed = 0

        pass

    def step(self, action):
        # given current obs and action returns the next observation, the reward, done and optionally additional info
        self.character.take_action(action)

        # compute next obs
        self.current_obs = self.character.get_observation()
        self.update_state()

        # compute reward
        reward = 0
        contact = self.curr_state["contactPoints"]
        if action[2] == 1:
            reward -= 0.05
        if (3 in contact or 4 in contact or 5 in contact):
            reward -= 0.1
        if (self.achieved_maze()):
            reward += 1
        if (self.button_distance() != 0):
            reward += 1
        if self.curr_state["CharacterPosition"][2] <= self.room_manager.room_array[0].global_coord[
            2]:
            reward -= 0.5
        # compute done

        self.time_passed += 1 / 240  # 1/240 s = 1 time step
        done = False
        if self.time_passed == self.time_episode or self.character.has_fallen() or self.achieved_maze():
            done = True
        return self.current_obs, reward, done, {}

    def reset(self):
        # Initial position : in front of the first window
        # these paremeters will need a change following the change of the room's or albert's characteristics
        # since these positions are those of the center of mass
        self.character.room_manager.room_array[self.character.actual_room].reset_room(self.character)

        # ENFAIT PAS FAIRE LE TRUC JUSTE EN HAUT :
        # RANDOMISER L'ORIENTATION INITIALE ET RANDOMISER LA POS INITIALE ( I COMPRIS ENTRE 1 ET 5 ET J ENTRE 1 ET 3 IL ME SEMBLE )
        xAlb = self.rng.uniform(1, 5)
        yAlb = self.rng.uniform(1, 3)
        zAlb = 0.75

        oriEuler = [0, 0, self.rng.uniform(-np.pi, np.pi)]

        self.character.reset_position_orientation([xAlb, yAlb, zAlb], oriEuler)

        self.current_obs = self.character.get_observation()

        self.time_passed = 0

        return self.current_obs

    def render(self):

        p.stepSimulation()
        time.sleep(1 / 240)

    def close(self):
        p.disconnect()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def button_distance(self):
        n = len(self.character.current_state["buttonsState"])
        if self.prev_state == None:
            return 0

        distance = sum([np.abs(self.curr_state["buttonsState"][i] - self.prev_state["buttonsState"][i]) for i in range(n)])
        return distance

    def achieved_maze(self):
        char_pos = self.curr_state["CharacterPosition"]
        door_pos = self.curr_state["doorPosition"]
        dist = np.sqrt(sum([(char_pos[i] - door_pos[i]) ** (2) for i in range(2)]))
        return (dist < 0.5)

    def update_state(self):
        self.curr_state = self.character.current_state
        self.prev_state = self.character.get_previous_state()

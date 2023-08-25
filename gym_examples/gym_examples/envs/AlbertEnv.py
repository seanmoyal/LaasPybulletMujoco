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
from Enums import JumpType,ObjectType

class AlbertEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, character=1):

        # Caracs "visibles"
        # initialisation
        physics_client = p.connect(p.GUI)  # p.DIRECT si pas besoin de graphisme
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -100)
        plane_id = p.loadURDF("plane.urdf")

        self.room_manager = RoomManager()
        # création et ajout du niveau 1
        base_cube = Cube()
        button = Button()
        door = Door()
        self.room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
        self.room_manager.room_array[0].build_basic_room(door)
        self.room_manager.room_array[0].build_button(button, 3, 4, 0)
        self.character = AlbertCube(self.room_manager)

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

        # Observation_space : 21 * tuple(objType,distance) pour chaque Rayon
        self.observation_space = Box(
            low=np.concatenate((np.array([0 for _ in range(105)]), np.array([0 for _ in range(105)]))),
            high=np.concatenate([np.array([5 for _ in range(105)]), np.array([10 for _ in range(105)])]), shape=(210,))

        # Action_space : [rotate,move,jump] avec f possible seulement si z=zsol
        self.action_space = MultiDiscrete(np.array([3, 3, 2]))
        # [[rotGauche,rotDroite],[reculer,pas bouger,avancer],[sautArriere,saut,sautAvant]]

        # RNG :
        self.rng = default_rng()

        # Observation courante
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
        if action[2] == JumpType.JUMP:
            reward -= 0.05
        if (ObjectType.WALL in contact or ObjectType.FENCE in contact or ObjectType.IBLOCK in contact):
            reward -= 0.1
        if (self.achieved_maze()):
            reward += 1
        if (self.button_distance() != 0):
            reward += 1
        if self.curr_state["CharacterPosition"][2] <= self.room_manager.room_array[0].global_coord[
            2]:  # a changer pr que ce soit qu'une fois ( quand il tombe )
            reward -= 0.5
        # compute done

        self.time_passed += 1 / 240  # à suposer qu'un step corresponde à 1/240 eme de seconde
        done = False
        if self.time_passed == self.time_episode or self.character.has_fallen() or self.achieved_maze():
            done = True
        return self.current_obs, reward, done, {}  # dictionnaire vide à la fin, pas important ici

    def reset(self):
        # position initiale : devante la première fenêtre
        # faudra changer ces params selon la taille d'albert et de la salle
        # car les positions correspondet a la position du centre de masse
        self.character.room_manager.room_array[self.character.actual_room].reset_room(self.character)
        xAlb = 3
        yAlb = 1
        zAlb = 0
        # ENFAIT PAS FAIRE LE TRUC JUSTE EN HAUT :
        # RANDOMISER L'ORIENTATION INITIALE ET RANDOMISER LA POS INITIALE ( I COMPRIS ENTRE 1 ET 5 ET J ENTRE 1 ET 3 IL ME SEMBLE )
        xAlb = self.rng.uniform(1, 5)
        yAlb = self.rng.uniform(1, 3)
        zAlb = 0.75

        oriEuler = [0, 0, self.rng.uniform(-np.pi, np.pi)]

        # A CHANGER ICI PSQ ON UTILISE PYBULLET ET FAUT QUE CA MARCHE PARTOUT
        self.character.reset_pos_ori([xAlb, yAlb, zAlb], oriEuler)

        self.current_obs = self.character.get_observation()
        # faire une requete pour chopper les raycasts

        # le temps repart à 0
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
        return (dist < 0.5)  # pour l'instant 0.5 mais en vrai dépend de la dim de la sortie et du character

    def update_state(self):
        self.curr_state = self.character.current_state
        self.prev_state = self.character.get_previous_state()

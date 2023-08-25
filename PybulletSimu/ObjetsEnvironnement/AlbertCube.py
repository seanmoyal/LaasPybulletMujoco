import pybullet as p
import numpy as np
from PybulletSimu.ObjetsEnvironnement.Cube import Cube
from Enums import TurnType,JumpType,MoveType,ObjectType

class AlbertCube(Cube):

    def __init__(self, room_manager):
        super().__init__(h_extents=[0.25, 0.25, 0.25])
        self.actual_room = 0
        self.room_manager = room_manager
        self.id = self.create_cube(pos=[2, 2, 1], orient_euler=[0, 0, 0], mass=1)

        self.time = 0

        # DEFINITION DU STATE ( albert n'y a pas acces)
        self.memory_state = []
        self.current_state = self.get_current_state()

        self.memory_observation = []  # longueur max de 5

        # y'a moyen que je passe tout ca en global
        self.count = 0  # pour le saut
        self.x_factor = 0  # pour le saut
        self.still_jumping = False  # pour le saut
        self.jumping = False  # pour le saut
        self.ori_jump = 0  # pour le saut

    def has_fallen(self):
        pos, ori = p.getBasePositionAndOrientation(self.id)
        return pos[2] < self.room_manager.room_array[self.actual_room].global_coord[2]

    def add_time(self, step):
        self.time += step

    def has_time(self):
        if self.time >= 4:
            self.room_manager.room_array[self.actual_room].reset_room(self)

    def reset_time(self):
        self.time = 0

    def create_cube(self, pos, orient_euler, mass):
        room_coord = self.room_manager.room_array[self.actual_room].global_coord
        newPos = [pos[0] + room_coord[0], pos[1] + room_coord[1], pos[2] + room_coord[2]]
        return super().create_cube(newPos, orient_euler, mass)

    def reset_pos_ori(self, pos, ori_euler):
        ori_quaternion = p.getQuaternionFromEuler(ori_euler)
        room_coord = self.room_manager.room_array[self.actual_room].global_coord
        new_pos = [pos[0] + room_coord[0], pos[1] + room_coord[1], pos[2] + room_coord[2]]
        p.resetBasePositionAndOrientation(self.id, new_pos, ori_quaternion)

    # hasSucceded c'est de la triche
    def has_succeded(self):
        char_pos, ori = p.getBasePositionAndOrientation(self.id)
        room = self.room_manager.room_array[self.actual_room]
        end_pos_j = room.global_coord[1] + room.width
        if char_pos[1] > end_pos_j:
            room.door_array[1].close(room.door_array[0])
            self.actual_room += 1
            self.reset_time()

    def raycasting(self):
        cube_pos, ori = p.getBasePositionAndOrientation(self.id)
        cube_ori = p.getEulerFromQuaternion(ori)
        matrice_ori = euler_to_rotation_matrix(cube_ori)
        ray_length = 5
        cube_pos_array = np.array([cube_pos for _ in range(7)])

        # départ des angles :
        dep_angles_yaw = -35 * np.pi / 180
        dep_angles_pitch = -10 * np.pi / 180
        # Pas yaw pour 70°
        step_yaw = 70 / 6
        step_yaw_rad = step_yaw * np.pi / 180

        # pas pitch pour 70°
        step_pitch = 20 / 2
        step_pitch_rad = step_pitch * np.pi / 180

        # rayVec1 : premier rayon droit devant le cube
        ray_vects = []
        for i in range(3):
            for n in range(7):
                base_ray = [np.cos(n * step_yaw_rad + dep_angles_yaw) * np.cos(i * step_pitch_rad + dep_angles_pitch),
                            np.sin(n * step_yaw_rad + dep_angles_yaw), np.sin(i * step_pitch_rad + dep_angles_pitch)]
                norm_ray = np.linalg.norm(base_ray)
                ray_vects.append(np.dot(matrice_ori, np.array(
                    [(base_ray[0] / norm_ray * ray_length + cube_pos[0]),
                     (ray_length * base_ray[1] / norm_ray + cube_pos[1]),
                     (ray_length * base_ray[2] / norm_ray + cube_pos[2])
                     ]
                )
                                        ))

        # tracé du rayon / faudra changer avec rayTestBatch() quand on aura plus de rayons
        contact_results = []
        for n in range(21):
            contact_results.append(p.rayTest(cube_pos, ray_vects[n]))

        # dans les resultats [0] : hitObjectId // [3] hit position
        # faudra changer les coordonnées de globales à locales
        """
        for n in range(21):
            if contact_results[n][0][0] != -1:
                p.addUserDebugLine(cube_pos, contact_results[n][0][3], [0, 1, 0], 2, 0.05)
            else:
                p.addUserDebugLine(cube_pos, ray_vects[n], [1, 0, 0], 2, 0.05)
        """

        return contact_results

    def jump(self, jump, move):
        i = 1000  # force du jump sur un pas
        move_x = 0
        if move == MoveType.BACKWARD_MOVE:
            move_x = -1
        elif move == MoveType.FORWARD_MOVE:
            move_x = 1
        if jump == JumpType.JUMP:
            self.still_jumping = True
            if len(p.getContactPoints(self.id)) != 0:
                self.jumping = True

        new_ori = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id)[1])[2]

        if self.jumping:
            self.x_factor = move_x
            self.ori_jump = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.id)[1])[2]
            impulse = [500 * self.x_factor, 0, i]
            p.applyExternalForce(self.id, -1, impulse, [0, 0, 0], p.LINK_FRAME)
            self.count += 1
            jumping = False
        elif (not self.jumping and self.still_jumping):
            if (self.count >= 1 and self.count <= 100):
                impulse = [500 * self.x_factor * np.cos(new_ori - self.ori_jump),
                           -500 * self.x_factor * np.sin(new_ori - self.ori_jump), i]
                p.applyExternalForce(self.id, -1, impulse, [0, 0, 0], p.LINK_FRAME)
                self.count += 1
            elif (len(p.getContactPoints(self.id)) == 0):
                impulse = [500 * self.x_factor * np.cos(new_ori - self.ori_jump),
                           -500 * self.xFactor * np.sin(new_ori - self.ori_jump), -i]
                p.applyExternalForce(self.id, -1, impulse, [0, 0, 0], p.LINK_FRAME)
            elif (len(p.getContactPoints(self.id)) != 0):  # a changer pour pas qu'il rebondisse sur les murs
                self.x_factor = 0
                self.still_jumping = False
                self.count = 0

    def yaw_turn(self, rotate):
        move_z = 0
        if rotate == TurnType.LEFT_TURN:
            move_z = -1
        elif rotate == TurnType.RIGHT_TURN:
            move_z = 1
        angular_velocity = [0, 0, 10 * move_z]  # mz=1/0/-1
        p.resetBaseVelocity(self.id, angularVelocity=angular_velocity)

    def move(self, move):  # AJOUTER LA CONDITION SUR S4IL EST DANS LES AIRS OU PAS
        move_x = 0
        if move == MoveType.BACKWARD_MOVE:
            move_x = -1
        elif move == MoveType.FORWARD_MOVE:
            move_x = 1
        linear_velocity = [move_x * 250, 0, 0]
        p.applyExternalForce(self.id, -1, linear_velocity, [0, 0, 0], p.LINK_FRAME)

    def take_action(self, action):  # 1: rotate, 2 : move, 3 : jump
        rotate = action[0]
        move = action[1]
        jump = action[2]
        self.yaw_turn(rotate)
        if ObjectType.FLOOR in self.current_state["contactPoints"]:
            self.move(move)
            self.jump(jump, move)
        self.current_state = self.get_current_state()

    def get_id(self):
        return self.id

    def get_observation(self):
        contact_results = self.raycasting()
        current_observation = np.empty(42)
        for i in range(len(contact_results)):
            room = self.room_manager.room_array[self.actual_room]
            if contact_results[i][0][0] == 0 or contact_results[i][0][0] == -1:
                current_observation[21 + i] = 10  # à changer en la distance du rayon
                current_observation[i] = ObjectType.NONE
            else:
                type = self.check_type(contact_results[i][0][0], room)
                distance = self.calc_distance(contact_results[i][0][0])
                current_observation[21 + i] = distance
                current_observation[i] = type

        # POUR L INSTANT ON VIRE Z DE L OBSERVATION

        # z = p.getBasePositionAndOrientation(self.id)[0][2]
        # current_observation.append(z)
        self.add_to_memory_observation(current_observation)
        obs = self.flat_memory()
        print(obs)
        return obs

    def check_type(self, id, room):
        buttons = room.buttons_array.keys()
        if id in buttons:
            return ObjectType.BUTTON

        if id in room.floor_array:
            return ObjectType.FLOOR

        if id in room.wall_array:
            return ObjectType.WALL

        fences = room.fences_array.keys()
        if id in fences:
            return ObjectType.FENCE

        iblocks = room.iblocks_array.keys()
        if id in iblocks:
            return ObjectType.IBLOCK

    def calc_distance(self, id):
        pos_object = p.getBasePositionAndOrientation(id)[0]
        pos_albert = p.getBasePositionAndOrientation(self.id)[0]

        distance = np.sqrt(sum([(pos_albert[i] - pos_object[i]) ** (2) for i in range(3)]))

        return distance

    def add_to_memory_observation(self, current_observation):  # A CHANGER CA VA DEVENIR CURRENT
        if len(self.memory_observation) < 5:
            self.memory_observation.append(current_observation)
        else:
            self.memory_observation[0] = self.memory_observation[1]
            self.memory_observation[1] = self.memory_observation[2]
            self.memory_observation[2] = self.memory_observation[3]
            self.memory_observation[3] = self.memory_observation[4]
            self.memory_observation[4] = current_observation

    def add_to_memory_state(self, current_state):
        if len(self.memory_state) < 5:
            self.memory_state.append(current_state)
        else:
            self.memory_state[0] = self.memory_state[1]
            self.memory_state[1] = self.memory_state[2]
            self.memory_state[2] = self.memory_state[3]
            self.memory_state[3] = self.memory_state[4]
            self.memory_state[4] = current_state

    def get_previous_state(self):
        if (len(self.memory_state) <= 1):
            return None
        return self.memory_state[len(self.memory_state) - 2]

    def get_current_state(self):
        room = self.room_manager.room_array[self.actual_room]
        current_state = {}
        pos_albert = p.getBasePositionAndOrientation(self.id)[0]
        buttons = room.buttons_array.values()
        buttons = binarize(buttons)
        door = np.prod(buttons)
        door_pos = p.getBasePositionAndOrientation(room.door_array[0])[0]

        current_state["CharacterPosition"] = [pos_albert[0], pos_albert[1], pos_albert[2]]
        current_state["doorState"] = door
        current_state["doorPosition"] = [door_pos[0], door_pos[1]]

        current_state["buttonsState"] = [buttons[i] for i in range(len(buttons))]

        # add contactpoints
        contact_points = p.getContactPoints(self.id)

        if len(contact_points) == 0:
            current_state["contactPoints"] = [0, 0, 0, 0, 0, 0]
        else:
            contact_types = []
            ids = []
            for i in range(len(contact_points)):
                id = contact_points[i][2]
                type = self.check_type(id, self.room_manager.room_array[self.actual_room])
                if id not in ids:
                    contact_types.append(type)
                    ids.append(id)
                if type == ObjectType.BUTTON:
                    pushed_button = self.room_manager.room_array[0].buttons_array.get(id)
                    if (pushed_button.is_pressed == False):
                        pushed_button.got_pressed(id)
            while (len(contact_types) < 6):
                contact_types.append(ObjectType.NONE)
            current_state["contactPoints"] = contact_types

        self.room_manager.room_array[self.actual_room].check_buttons_pushed()

        self.add_to_memory_state(current_state)

        return current_state

    def flat_memory(self):
        obs = np.zeros(210)
        for i in range(len(self.memory_observation)):
            for j in range(42):
                if j < 21:
                    obs[i * 21 + j] = self.memory_observation[i][j]
                else:
                    obs[105 + i * 21 + (j - 21)] = self.memory_observation[i][j]
        return obs


def binarize(buttons):
    list = []
    for button in buttons:
        if button.is_pressed:
            list.append(1)
        else:
            list.append(0)
    return list


def euler_to_rotation_matrix(euler):
    # Convert Euler angles to rotation matrix
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    cos_r, sin_r = np.cos(roll), np.sin(roll)
    cos_p, sin_p = np.cos(pitch), np.sin(pitch)
    cos_y, sin_y = np.cos(yaw), np.sin(yaw)

    rotation_matrix = np.array([
        [cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r, cos_y * sin_p * cos_r + sin_y * sin_r],
        [sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r, sin_y * sin_p * cos_r - cos_y * sin_r],
        [-sin_p, cos_p * sin_r, cos_p * cos_r]
    ])

    return rotation_matrix

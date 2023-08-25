import pybullet
import pybullet as p
import time
import os
import pathlib
import pybullet_data
from ObjetsEnvironnement.RoomManager import RoomManager
from ObjetsEnvironnement.Cube import Cube
from ObjetsEnvironnement.Door import Door
from ObjetsEnvironnement.IBlock import IBlock
from ObjetsEnvironnement.Button import Button
from ObjetsEnvironnement.Fence import Fence
from ObjetsEnvironnement.Room import Room
from ObjetsEnvironnement.Character import Character

from XmlConversionDirectory.xmlConverter import xml_room_manager_pybullet


def move():  # pour bouger dans l'espace
    keys = p.getKeyboardEvents()
    cam = p.getDebugVisualizerCamera()
    # Keys to change camera
    if keys.get(p.B3G_DOWN_ARROW):  # D
        xyz = cam[11]
        x = float(xyz[0]) + 0.125
        y = xyz[1]
        z = xyz[2]
        p.resetDebugVisualizerCamera(cameraYaw=cam[8], cameraPitch=cam[9], cameraDistance=cam[10],
                                     cameraTargetPosition=[x, y, z])
    if keys.get(p.B3G_UP_ARROW):  # A
        xyz = cam[11]
        x = float(xyz[0]) - 0.125
        y = xyz[1]
        z = xyz[2]
        p.resetDebugVisualizerCamera(cameraYaw=cam[8], cameraPitch=cam[9], cameraDistance=cam[10],
                                     cameraTargetPosition=[x, y, z])
    if keys.get(p.B3G_RIGHT_ARROW):  # C
        xyz = cam[11]
        x = xyz[0]
        y = float(xyz[1]) + 0.125
        z = xyz[2]
        p.resetDebugVisualizerCamera(cameraYaw=cam[8], cameraPitch=cam[9], cameraDistance=cam[10],
                                     cameraTargetPosition=[x, y, z])
    if keys.get(p.B3G_LEFT_ARROW):  # F
        xyz = cam[11]
        x = xyz[0]
        y = float(xyz[1]) - 0.125
        z = xyz[2]
        p.resetDebugVisualizerCamera(cameraYaw=cam[8], cameraPitch=cam[9], cameraDistance=cam[10],
                                     cameraTargetPosition=[x, y, z])


physics_client = p.connect(p.GUI)  # p.DIRECT si pas besoin de graphisme
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -100)
plane_id = p.loadURDF("plane.urdf")
start_pos = [0, 0, 100]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
# absPath=pathlib.Path(__file__).parent.resolve()
# absPath.replace(os.sep, '/')
# pathUrdf="/UrdfDirectory/"
# print("path : ",absPath)
# pathCube=absPath+pathUrdf+"Cube.urdf"
# boxId = p.loadURDF("C:/Users/moyal/PycharmProjects/testEnviSim/PybulletSimu/UrdfDirectory/Cube.urdf",startPos, startOrientation)
base_cube = Cube()
iblock_cube = IBlock(Cube(color=[0, 0, 0, 1]), 1)
iblock_cube_2 = IBlock(Cube(color=[0, 0, 0, 1]), 2)
iblock_cube_3 = IBlock(Cube(color=[0, 0, 0, 1]), 3)
button = Button()
door = Door()
fence = Fence(2.5, .5)
fence_2 = Fence(2.5, 1.5)

# p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# SOL
# déclaration du room Manager
room_manager = RoomManager()

# création et ajout du niveau 1
room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
room_manager.room_array[0].build_basic_room(door)
room_manager.room_array[0].build_button(button, 3, 4, 0)
xml_room_manager_pybullet(room_manager, "Room2bis")

# création et ajout du niveau 2
room_manager.add_room(Room(6, 11, 3, base_cube, 8, -6, 0.5))
room_manager.room_array[1].build_basic_room(door)
room_manager.room_array[1].build_button(button, 2, 3, 0)
room_manager.room_array[1].build_button(button, 4, 7, 0)
room_manager.room_array[1].build_fence(fence, 3, 5)

# création et ajout du niveau 3
room_manager.add_room(Room(6, 11, 3, base_cube, 8, -6, 0.5))
room_manager.room_array[2].build_basic_room(door)
room_manager.room_array[2].build_iblock(iblock_cube, 4, 6)
room_manager.room_array[2].build_button(button, 4, 6, 1)

# création et ajout du niveau 4
room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
room_manager.room_array[3].build_basic_room(door)
room_manager.room_array[3].build_iblock(iblock_cube, 3, 3)
room_manager.room_array[3].build_button(button, 3, 3, 1)
room_manager.room_array[3].build_iblock(iblock_cube_2, 5, 5)
room_manager.room_array[3].build_button(button, 5, 5, 2)

# création et ajout du niveau 5
room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
room_manager.room_array[4].build_basic_room(door)
room_manager.room_array[4].build_iblock(iblock_cube, 1, 1)
room_manager.room_array[4].build_button(button, 1, 1, 1)
room_manager.room_array[4].build_iblock(iblock_cube_2, 1, 3.5)
room_manager.room_array[4].build_button(button, 1, 3.5, 2)
room_manager.room_array[4].build_iblock(iblock_cube_2, 4, 4.5)
room_manager.room_array[4].build_button(button, 4, 4.5, 2)
room_manager.room_array[4].build_iblock(iblock_cube_2, 2.5, 6.5)
room_manager.room_array[4].build_button(button, 2.5, 6.5, 2)
room_manager.room_array[4].build_iblock(iblock_cube_3, 1, 9)
room_manager.room_array[4].build_button(button, 1, 9, 3)
room_manager.room_array[4].build_fence(fence, 3, 3)
room_manager.room_array[4].build_fence(fence_2, 3, 5.8)

# alignement des niveaux
room_manager.align_rooms()

# On gère ici les Collisions
character = Character(p.loadURDF("r2d2.urdf", [-1.5, 2.5, 1.7]), room_manager)
p.setCollisionFilterGroupMask(character.id, -1, 0, 0)
print(character.id)

# regarder sur quel niveau l'ia se trouve et boucler dessus tant que l'ia n'a pas trouvé la sortie -> 6

# le step simu
step = 1. / 240.
for i in range(10000):
    move()
    p.stepSimulation()
    character.add_time(step)
    character.has_time()
    room_manager.room_array[0].check_buttons_pushed()
    contact_points = p.getContactPoints(bodyA=character.id)  # ajouter en body A le personnage
    character.has_fallen()
    character.has_succeded()
    for point in contact_points:  # ici maintenant faut regarder dans quel niveau l'ia se trouve
        body_id = point[2]
        if body_id in room_manager.room_array[0].buttons_array.keys():
            pushed_button = room_manager.room_array[0].buttons_array.get(body_id)
            if not pushed_button.is_pressed:
                pushed_button.got_pressed(body_id)

            # on change la couleur des bouttons passés dessus à gris
            # en vrai à changer pour modifier la room, faut que le buttonsArray soit dans la room
    time.sleep(step)

p.disconnect()

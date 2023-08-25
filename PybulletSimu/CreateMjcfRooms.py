import pybullet as p
import pybullet_data
from ObjetsEnvironnement.RoomManager import RoomManager
from ObjetsEnvironnement.Cube import Cube
from ObjetsEnvironnement.Door import Door
from ObjetsEnvironnement.IBlock import IBlock
from ObjetsEnvironnement.Button import Button
from ObjetsEnvironnement.Fence import Fence
from ObjetsEnvironnement.Room import Room

from XmlConversionDirectory.xmlConverter import xml_room_manager_pybullet

physics_client = p.connect(p.GUI)  # p.DIRECT si pas besoin de graphisme
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -100)
plane_id = p.loadURDF("plane.urdf")
start_pos = [0, 0, 100]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])
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


#Choisir où mettre cette ligne pour choisir sa room
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
p.disconnect()

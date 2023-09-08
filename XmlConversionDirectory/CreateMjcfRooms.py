import pybullet as p
import pybullet_data
from PybulletSimu.ObjectsEnvironment.RoomManager import RoomManager
from PybulletSimu.ObjectsEnvironment.Cube import Cube
from PybulletSimu.ObjectsEnvironment.Door import Door
from PybulletSimu.ObjectsEnvironment.IBlock import IBlock
from PybulletSimu.ObjectsEnvironment.Button import Button
from PybulletSimu.ObjectsEnvironment.Fence import Fence
from PybulletSimu.ObjectsEnvironment.Room import Room

from XmlConversionDirectory.xmlConverter import xml_room_manager_pybullet

def add_room_by_number(room_manager,room_index,file_name):
    if room_index==0:
        room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
        room_manager.room_array[0].build_basic_room(door)
        room_manager.room_array[0].build_button(button, 3, 4, 0)
    elif room_index==1:
        room_manager.add_room(Room(6, 11, 3, base_cube, 8, -6, 0.5))
        room_manager.room_array[0].build_basic_room(door)
        room_manager.room_array[0].build_button(button, 2, 3, 0)
        room_manager.room_array[0].build_button(button, 4, 7, 0)
        room_manager.room_array[0].build_fence(fence, 3, 5)
    elif room_index ==2:
        room_manager.add_room(Room(6, 11, 3, base_cube, 8, -6, 0.5))
        room_manager.room_array[0].build_basic_room(door)
        room_manager.room_array[0].build_iblock(iblock_cube, 4, 6)
        room_manager.room_array[0].build_button(button, 4, 6, 1)
    elif room_index==3:
        room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
        room_manager.room_array[0].build_basic_room(door)
        room_manager.room_array[0].build_iblock(iblock_cube, 3, 3)
        room_manager.room_array[0].build_button(button, 3, 3, 1)
        room_manager.room_array[0].build_iblock(iblock_cube_2, 5, 5)
        room_manager.room_array[0].build_button(button, 5, 5, 2)
    else:
        room_manager.add_room(Room(6, 11, 3, base_cube, -5.5, -1.5, 0.5))
        room_manager.room_array[0].build_basic_room(door)
        room_manager.room_array[0].build_iblock(iblock_cube, 1, 1)
        room_manager.room_array[0].build_button(button, 1, 1, 1)
        room_manager.room_array[0].build_iblock(iblock_cube_2, 1, 3.5)
        room_manager.room_array[0].build_button(button, 1, 3.5, 2)
        room_manager.room_array[0].build_iblock(iblock_cube_2, 4, 4.5)
        room_manager.room_array[0].build_button(button, 4, 4.5, 2)
        room_manager.room_array[0].build_iblock(iblock_cube_2, 2.5, 6.5)
        room_manager.room_array[0].build_button(button, 2.5, 6.5, 2)
        room_manager.room_array[0].build_iblock(iblock_cube_3, 1, 9)
        room_manager.room_array[0].build_button(button, 1, 9, 3)
        room_manager.room_array[0].build_fence(fence, 3, 3)
        room_manager.room_array[0].build_fence(fence_2, 3, 5.8)

    xml_room_manager_pybullet(room_manager, file_name)



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

save_mjcf_file_name = "Room2bis"
add_room_by_number(room_manager,room_index=0,file_name=save_mjcf_file_name)

# alignement des niveaux
room_manager.align_rooms()
p.disconnect()



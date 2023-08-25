import os
import xml.etree.ElementTree as ET
import pybullet as p


def xml_floor(room, pos, euler, name):
    body = ET.SubElement(room, 'body', name=name, pos=list_to_string(pos), euler=list_to_string(euler))
    ET.SubElement(body, 'geom', type='box', size=".5 .5 .5", pos="0 0 0", rgba="1 1 1 1", friction="0.5 0.005 0.0001")
    # tree=ET.ElementTree(root)
    # tree.write('Cube.xml')


def xml_wall(room, pos, euler, name):
    body = ET.SubElement(room, 'body', name=name, pos=list_to_string(pos), euler=list_to_string(euler))
    ET.SubElement(body, 'geom', type='box', size=".5 .5 .5", pos="0 0 0", rgba="1 1 1 1", friction="0.1 0.005 0.0001")
    # tree=ET.ElementTree(root)
    # tree.write('Cube.xml')


def xml_fence(room, pos, euler, depth, height, name):  # a stringifier pour la size
    size = list_to_string([depth, 0.1, height])
    body = ET.SubElement(room, 'body', name=name, pos=list_to_string(pos), euler=list_to_string(euler))
    ET.SubElement(body, 'geom', type='box', size=size, pos="0 0 0", rgba="0.5 0.5 0.5 1", friction="1 0.005 0.0001")
    # tree=ET.ElementTree(root)
    # tree.write('Fence.xml')


def xml_door(room, pos, euler, name):
    body = ET.SubElement(room, 'body', name=name, pos=list_to_string(pos), euler=list_to_string(euler))
    ET.SubElement(body, 'geom', type='box', size=".5 .5 .5", pos="0 0 0", rgba="0 0 0 1", friction="1 0.005 0.0001")
    # tree=ET.ElementTree(root)
    # tree.write('Door.xml')


def xml_iblock(room, pos, euler, name, height=1):
    size = list_to_string([0.5, 0.5, height / 2])
    pos = [pos[0], pos[1], pos[2] - (height - 1) / 2]
    body = ET.SubElement(room, 'body', name=name, pos=list_to_string(pos), euler=list_to_string(euler))
    ET.SubElement(body, 'geom', type='box', size=size, pos="0 0 0", rgba="0 0 0 1", friction="1 0.005 0.0001")
    # tree=ET.ElementTree(root)
    # tree.write('IBlock.xml')


def xml_button(room, pos, euler, name):  # changer la size z
    body = ET.SubElement(room, 'body', name=name, pos=list_to_string(pos), euler=list_to_string(euler))
    ET.SubElement(body, 'geom', type='box', size=".5 .5 .02", pos="0 0 0", rgba="0 1 0 1", friction="0.1 0.005 0.0001")
    # tree=ET.ElementTree(root)
    # tree.write('Button.xml')


def xml_room_pybullet(worldbody, room, name):
    pos_room = room.global_coord

    room_tree = ET.SubElement(worldbody, 'body', name=name, pos=list_to_string(pos_room))
    x, y, l = room.global_coord[0], room.global_coord[1], room.global_coord[2]
    buttons_array = room.buttons_array
    floor_array = room.floor_array
    wall_array = room.wall_array
    iblocks_array = room.iblocks_array
    fences_array = room.fences_array
    door_array = room.door_array
    i = 0
    for id in buttons_array.keys():
        world_pos, quat = p.getBasePositionAndOrientation(id)
        euler = p.getEulerFromQuaternion(quat)
        pos = [world_pos[0] - x, world_pos[1] - y, world_pos[2] - l]
        name = 'button{}'.format(i)
        xml_button(room_tree, pos, euler, name)
        i += 1
    i = 0
    for id in floor_array:
        world_pos, quat = p.getBasePositionAndOrientation(id)
        euler = p.getEulerFromQuaternion(quat)
        pos = [world_pos[0] - x, world_pos[1] - y, world_pos[2] - l]
        name = 'floor{}'.format(i)
        xml_floor(room_tree, pos, euler, name)
        i += 1
    i = 0
    for id in wall_array:
        world_pos, quat = p.getBasePositionAndOrientation(id)
        euler = p.getEulerFromQuaternion(quat)
        pos = [world_pos[0] - x, world_pos[1] - y, world_pos[2] - l]
        name = 'wall{}'.format(i)
        xml_wall(room_tree, pos, euler, name)
        i += 1
    i = 0
    for id in iblocks_array.keys():
        world_pos, quat = p.getBasePositionAndOrientation(id)
        euler = p.getEulerFromQuaternion(quat)
        pos = [world_pos[0] - x, world_pos[1] - y, world_pos[2] - l]
        name = 'iblock{}'.format(i)
        xml_iblock(room_tree, pos, euler, name, iblocks_array[id].height)  # AAAAAAAAAAAAAA
        i += 1
    i = 0
    for id in fences_array.keys():
        world_pos, quat = p.getBasePositionAndOrientation(id)
        euler = p.getEulerFromQuaternion(quat)
        pos = [world_pos[0] - x, world_pos[1] - y, world_pos[2] - l]
        name = 'fence{}'.format(i)
        xml_fence(room_tree, pos, euler, fences_array[id].depth, fences_array[id].height, name)
        i += 1
    i = 0

    world_pos, quat = p.getBasePositionAndOrientation(door_array[0])
    euler = p.getEulerFromQuaternion(quat)
    pos = [world_pos[0] - x, world_pos[1] - y, world_pos[2] - l]
    xml_door(room_tree, pos, euler, 'door')


def xml_room_manager_pybullet(room_manager, file_name):
    # Create the root element for the MJCF file

    root = ET.Element('mujoco', model='plane_field')

    # Create the compiler element
    compiler = ET.SubElement(root, 'compiler', angle='radian', coordinate='local')

    # Create the option element
    option = ET.SubElement(root, 'option', gravity='0 0 -10 ', timestep="0.001")

    # Create the worldbody element
    worldbody = ET.SubElement(root, 'worldbody')

    # Create the geom element for the plane field
    geom = ET.SubElement(worldbody, 'geom', type='plane', name='field', size='40 40 0.01', pos='0 30 -0.5')
    i = 1
    for room in room_manager.room_array:
        name = "room" + str(i)
        xml_room_pybullet(worldbody, room, name)
        i += 1
    tree = ET.ElementTree(root)
    project_path = get_absolute_path_project("testEnviSim").replace('\\', '/')
    file = project_path+'/xmlDirectory/' + file_name + ".xml"
    tree.write(file)


def list_to_string(list):
    s = ""
    for l in list:
        s += " " + str(l)
    return s

def get_absolute_path_project(project_name):

    script_directory = os.path.dirname(os.path.abspath(__file__))

    current_directory = script_directory
    while current_directory != os.path.dirname(current_directory):
        if os.path.basename(current_directory) == project_name:
            return current_directory
        current_directory = os.path.dirname(current_directory)

    # If the specified directory is not found, return None
    return None
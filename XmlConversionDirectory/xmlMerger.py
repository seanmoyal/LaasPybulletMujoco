import os
import xml.etree.ElementTree as ET
import gym_albert_mujoco

def merge_mjcf_files(file_1, file_2, file_name):
    tree_1 = ET.parse(file_1)
    root_1 = tree_1.getroot()
    worldbody_1 = root_1.find('worldbody')

    tree_2 = ET.parse(file_2)
    root_2 = tree_2.getroot()
    worldbody_2 = root_2.find('worldbody')

    for child in worldbody_2:
        worldbody_1.append(child)
    mujoco_path = gym_albert_mujoco.__file__
    project_idx = mujoco_path.find("/gym_albert_mujoco")
    project_path = mujoco_path[:project_idx]
    xml_directory_path = project_path + "/xmlDirectory/"
    link = xml_directory_path + file_name + ".xml"
    tree_1.write(link)

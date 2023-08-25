import os
import xml.etree.ElementTree as ET

f1 = "C:/Users/moyal/PycharmProjects/testEnviSim/xmlDirectory/Test1.xml"
f2 = "C:/Users/moyal/PycharmProjects/testEnviSim/xmlDirectory/Actor.xml"


def merge_mjcf_files(file_1, file_2, file_name):
    tree_1 = ET.parse(file_1)
    root_1 = tree_1.getroot()
    worldbody_1 = root_1.find('worldbody')

    tree_2 = ET.parse(file_2)
    root_2 = tree_2.getroot()
    worldbody_2 = root_2.find('worldbody')

    for child in worldbody_2:
        worldbody_1.append(child)

    project_name = "testEnviSim"
    project_path = get_absolute_path_project(project_name).replace('\\', '/')
    xml_directory_path = project_path + "/xmlDirectory/"
    link = xml_directory_path + file_name + ".xml"
    tree_1.write(link)

def get_absolute_path_project(project_name):

    script_directory = os.path.dirname(os.path.abspath(__file__))

    current_directory = script_directory
    while current_directory != os.path.dirname(current_directory):
        if os.path.basename(current_directory) == project_name:
            return current_directory
        current_directory = os.path.dirname(current_directory)

    # If the specified directory is not found, return None
    return None
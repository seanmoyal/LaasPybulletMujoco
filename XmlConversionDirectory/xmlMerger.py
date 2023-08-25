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

    link = "C:/Users/moyal/PycharmProjects/testEnviSim/xmlDirectory/" + file_name + ".xml"
    tree_1.write(link)


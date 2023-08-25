import xml.etree.ElementTree as ET

# Create the root element for the MJCF file
root = ET.Element('mujoco', model='plane_field')

# Create the compiler element
compiler = ET.SubElement(root, 'compiler', angle='radian', coordinate='local')

# Create the option element
option = ET.SubElement(root, 'option', gravity='0 0 0 ')

# Create the worldbody element
worldbody = ET.SubElement(root, 'worldbody')

# Create the geom element for the plane field
geom = ET.SubElement(worldbody, 'geom', type='plane', name='field', size='10 10 0.01', pos='0 0 0')

# Create the ElementTree object and write it to a file
tree = ET.ElementTree(root)
print(tree)
tree.write('plane_field.xml')

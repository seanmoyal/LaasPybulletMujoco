import pybullet
import pybullet as p
import time
import os
import pathlib
import pybullet_data
physics_client = p.connect(p.GUI)# p.DIRECT si pas besoin de graphisme

start_pos = [0,0,100]
start_orientation = p.getQuaternionFromEuler([0,0,0])
box_id = p.loadURDF("C:/Users/moyal/PycharmProjects/testEnviSim/PybulletSimu/UrdfDirectory/Cube.urdf",start_pos, start_orientation)
p.resetBasePositionAndOrientation(box_id, start_pos, start_orientation)

for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cube_pos, cube_ori = p.getBasePositionAndOrientation(box_id)
print(cube_pos,cube_ori)
p.disconnect()

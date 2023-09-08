import pybullet as p

from PybulletSimu.ObjetsEnvironnement.Cube import Cube


class Door(Cube):
    def __init__(self):
        self.is_opened = False
        Cube.__init__(self, color=[0, 0, 0, 1])

    def open(self, id):
        self.is_opened = True
        pos, ori = p.getBasePositionAndOrientation(id)
        new_pos = [pos[0], pos[1], pos[2] + 1]
        p.resetBasePositionAndOrientation(id, new_pos, ori)

    def close(self, id):
        self.is_opened = False
        pos, ori = p.getBasePositionAndOrientation(id)
        new_pos = [pos[0], pos[1], pos[2] - 1]
        p.resetBasePositionAndOrientation(id, new_pos, ori)

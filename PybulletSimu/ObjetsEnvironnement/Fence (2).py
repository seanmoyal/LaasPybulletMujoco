from PybulletSimu.ObjetsEnvironnement.Cube import Cube


class Fence(Cube):

    def __init__(self, depth, height):
        self.depth = depth
        self.height = height
        Cube.__init__(self, h_extents=[depth, .05, height], color=[.5, .5, .5, 1])

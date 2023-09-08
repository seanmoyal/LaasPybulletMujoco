from PybulletSimu.ObjectsEnvironment.Cube import Cube


class Fence(Cube):  # classe d'une barrière pour empêcher l'ia de passer

    def __init__(self, depth, height):
        self.depth = depth
        self.height = height
        Cube.__init__(self, h_extents=[depth, .05, height], color=[.5, .5, .5, 1])

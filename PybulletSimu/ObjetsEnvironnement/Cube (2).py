import pybullet as p


class Cube:  # Mother Class for all elements of the simulation
    def __init__(self, h_extents=[0.5, 0.5, 0.5], color=[1, 1, 1, 1], specular_color=[1, 1, 1]):
        self.h_extents = h_extents
        self.color = color
        self.specular_color = specular_color
        self.vis_box_id = p.createVisualShape(p.GEOM_BOX, rgbaColor=self.color, specularColor=self.specular_color,
                                              halfExtents=self.h_extents)
        self.col_box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.h_extents)

    def create_cube(self, pos, orient_euler, mass):
        orient = p.getQuaternionFromEuler(orient_euler)
        box_id = p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=self.col_box_id,
                                   baseVisualShapeIndex=self.vis_box_id, basePosition=pos, baseOrientation=orient)
        return box_id

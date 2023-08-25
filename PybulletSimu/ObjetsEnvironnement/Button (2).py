from PybulletSimu.ObjetsEnvironnement.Cube import Cube
import pybullet as p
class Button(Cube):

    def __init__(self):
        self.is_pressed = False
        Cube.__init__(self,h_extents=[.5,.5,.02],color=[0,1,0,1])

    def create_button(self,pos,orient_euler,mass):
        orient=p.getQuaternionFromEuler(orient_euler)
        box_id = p.createMultiBody(baseMass = mass,baseCollisionShapeIndex = self.col_box_id, baseVisualShapeIndex = self.vis_box_id, basePosition = pos, baseOrientation = orient)
        return box_id

    def got_pressed(self,body_id):
        p.changeVisualShape(body_id, -1, rgbaColor=[.7, .7, .7,1])
        self.is_pressed=True


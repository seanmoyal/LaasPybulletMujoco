import pybullet as p
class Room:  # classe d'une chambre ( niveau )
    def __init__(self, depth, width, height, material, x, y, l):
        self.global_coord = [x, y, l]  # l=0.5 # Coordonées globales de la chambre
        self.buttons_array = {}
        self.floor_array = []
        self.wall_array = []
        self.iblocks_array = {}
        self.fences_array = {}
        self.door_array = []
        self.depth = depth
        self.width = width
        self.height = height
        self.material = material

        # DEFINITION DU STATE :

    def build_basic_room(self, Door):  # construction de la structure de la chambre et stockage des blocs dans une liste
        x, y, l = self.global_coord[0], self.global_coord[1], self.global_coord[2]
        for i in range(self.depth):
            for j in range(self.width):
                box_pos = [x + i, y + j, l]
                mass = 0
                box_id = self.material.create_cube(box_pos, [0, 0, 0], 0)
                self.floor_array.append(box_id)
                for z in range(self.height):  # MURS
                    if i == 0 or (j == 0 or j == 10):
                        if i == self.depth / 2 and (j == self.width - 1 or j == 0) and (z == 0):
                            if j == self.width - 1:
                                box_pos = [x + i, y + j, l + 1 + z]
                                self.build_door(Door, i, j, z)
                        else:
                            box_pos = [x + i, y + j, l + 1 + z]
                            box_id = self.material.create_cube(box_pos, [0, 0, 0], 0)
                            self.wall_array.append(box_id)

    def build_iblock(self, IBlock, i,
                     j):  # fonction de contruction d'un IBlock et stockage de son id dans un dictionnaire d'iblocs
        x, y, l = self.global_coord[0], self.global_coord[1], self.global_coord[2]
        h = IBlock.height
        for z in range(h):
            box_id = IBlock.material.create_cube([x + i, y + j, z + 1 + self.global_coord[2]], [0, 0, 0], 0)
            self.iblocks_array[box_id] = IBlock

    def build_button(self, Button, i, j,
                     z):  # fonction de contruction d'un boutton et stockage de son id dans un dictionnaire de bouttons
        x, y, l = self.global_coord[0], self.global_coord[1], self.global_coord[2]
        box_id = Button.create_cube([x + i, y + j, z + 0.02 + l + 0.5], [0, 0, 0], 0)
        self.buttons_array[box_id] = Button

    def build_fence(self, Fence, i,
                    j):  # fonction de contruction d'une barrière et stockage de son id dans un dictionnaire
        # on pourra ajouter l'orientation selon z après
        x, y, l = self.global_coord[0], self.global_coord[1], self.global_coord[2]
        box_id = Fence.create_cube([i + x, j + y, 0.5 + l + Fence.height / 2], [0, 0, 0], 0)
        self.fences_array[box_id] = Fence

    def build_door(self, Door, i, j,
                   z):  # fonction de contruction d'une barrière et stockage de son id dans un dictionnaire
        # on pourra ajouter l'orientation selon z après
        x, y, l = self.global_coord[0], self.global_coord[1], self.global_coord[2]
        box_id = Door.create_cube([i + x, j + y, l + 1 + z], [0, 0, 0], 0)
        self.door_array.append(box_id)
        self.door_array.append(Door)

    def change_global_coord(self, x, y,
                            l):  # fonction qui translate une chambre d'un endroit à un autre ( translation de tout ce qu'il y a dedans aussi)
        old_global_coord = self.global_coord
        self.global_coord = [x, y, l]
        translation = [x - old_global_coord[0], y - old_global_coord[1], l - old_global_coord[2]]

        for cube_id in self.floor_array:
            translate(cube_id, translation)

        for cube_id in self.wall_array:
            translate(cube_id, translation)

        for button_id in self.buttons_array.keys():
            translate(button_id, translation)

        for fence_id in self.fences_array.keys():
            translate(fence_id, translation)

        for iblock_id in self.iblocks_array.keys():
            translate(iblock_id, translation)

        translate(self.door_array[0], translation)

    def check_buttons_pushed(self):
        if not self.door_array[1].is_opened:
            a = False
            for button in self.buttons_array.values():
                if not button.is_pressed:
                    a = True
            if not a:
                self.door_array[1].open(self.door_array[0])

    def reset_room(self,
                   character):  # dans la vidéo chaque simu se termine après 10s, on appelera cette fo après 10 s de simu
        # Evidemment elle est à compléter
        for id_button in self.buttons_array.keys():
            p.changeVisualShape(id_button, -1, rgbaColor=[0, 1, 0, 1])
            self.buttons_array[id_button].is_pressed = False
        base_orientation = p.getQuaternionFromEuler([0, 0, 0])
        base_position = [self.global_coord[0] + self.depth / 2, self.global_coord[1] + 1, self.global_coord[2] + 1]
        p.resetBasePositionAndOrientation(character.id, base_position, base_orientation)


        if self.door_array[1].is_opened:
            self.door_array[1].close(self.door_array[0])


def translate(id,
              translation):  # fonction de translation utilisiée dans changeGlobalCoord(), translate un objet d'identifiant Id
    old_position, orientation = p.getBasePositionAndOrientation(id)

    new_position = [
        old_position[0] + translation[0],
        old_position[1] + translation[1],
        old_position[2] + translation[2]
    ]

    p.resetBasePositionAndOrientation(id, new_position, orientation)

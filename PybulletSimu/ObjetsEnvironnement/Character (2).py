import pybullet as p
class Character:

    def __init__(self,id,room_manager):
        self.id = id
        self.actual_room = 0
        self.room_manager=room_manager
        self.time=0

    def has_fallen(self):
        pos,ori = p.getBasePositionAndOrientation(self.id)
        if (pos[2] < self.room_manager.room_array[self.actual_room].global_coord[2]):
            self.room_manager.room_array[self.actual_room].reset_room(self)

    def add_time(self,step):
        self.time += step

    def has_time(self):
        if self.time>=4:
            self.room_manager.room_array[self.actual_room].reset_room(self)

    def reset_time(self):
        self.time =0

    def has_succeded(self):
        char_pos, ori = p.getBasePositionAndOrientation(self.id)
        room = self.room_manager.room_array[self.actual_room]
        end_pos_j =room.global_coord[1]+room.width
        if char_pos[1]>end_pos_j:
            room.door_array[1].close(room.door_array[0])
            self.actual_room += 1
            self.reset_time()


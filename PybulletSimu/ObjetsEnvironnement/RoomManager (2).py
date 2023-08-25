

class RoomManager: # class managing multiple rooms
    def __init__(self):
        self.room_array=[]

    def add_room(self,room):
        self.room_array.append(room)

    def align_rooms(self):
        n=len(self.room_array)
        if(n>1):
            x,y,z=self.room_array[0].global_coord[0],self.room_array[0].global_coord[1],self.room_array[0].global_coord[2]
            for i in range (1,n):
                self.room_array[i].change_global_coord(x,self.room_array[i-1].global_coord[1]+self.room_array[i-1].width,z)

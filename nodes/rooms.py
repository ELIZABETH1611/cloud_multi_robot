import numpy as np


class Check_room(object):
    """docstring for ."""

    def __init__(self):
        self.room_0_x =[-5,5]
        self.room_0_y =[-5,5]
        self.room_1_x =[-4,5]
        self.room_1_y =[5,12]
        self.room_2_x =[-15,-4]
        self.room_2_y =[5,12]
        self.room_3_x =[-15,-5]
        self.room_3_y =[-5,5]
        
    def check_room(self,x_robot,y_robot):
        if (self.room_0_x[1]>= x_robot >= self.room_0_x[0]) and\
           (self.room_0_y[1]>= y_robot >= self.room_0_y[0]):
           return 0
        elif (self.room_1_x[1]>= x_robot >= self.room_1_x[0]) and\
              (self.room_1_y[1]>= y_robot >= self.room_1_y[0]):
           return 1
        elif (self.room_2_x[1]>= x_robot >= self.room_2_x[0]) and\
              (self.room_2_y[1]>= y_robot >= self.room_2_y[0]):
           return 2
        elif (self.room_3_x[1]>= x_robot >= self.room_3_x[0]) and\
              (self.room_3_y[1]>= y_robot >= self.room_3_y[0]):
           return 3
        else:
            raise Exception("There is no room, check your data x: "+str(x_robot)+" y: "+str(y_robot))

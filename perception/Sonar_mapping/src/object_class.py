import numpy as np


class LOCATION_OF_OBJECT:
    # Constructor
    def __init__(self,name):
        
        # Variables
        self.name = name
        self.detected = False
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0
        self.queue = 0
        
    # Class functions
    def update(self,map_coordinates):
        if self.detected == True:
            #if map_coordinates[0] < (self.x_pos+1) and  map_coordinates[0] > (self.x_pos-1):
                #if map_coordinates[1] < (self.y_pos+1) and  map_coordinates[1] > (self.y_pos-1):
                    #self.x_pos = (self.x_pos) * 0.9 + (map_coordinates[0]) * 0.1
                    #self.y_pos = (self.y_pos) * 0.9 + (map_coordinates[1]) * 0.1
            self.x_pos = map_coordinates[0]
            self.y_pos = map_coordinates[1]

        elif self.detected == False:
            self.queue += 1
            if self.queue >= 5:
                self.detected = True
            self.x_pos += 0.2*(map_coordinates[0])
            self.y_pos += 0.2*(map_coordinates[1])



        
        
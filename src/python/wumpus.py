from math_utils import *
from graphics import *
from occupancy_grid import *
from colors import *
from search import *
from clock import *

class Wumpus:
    speed = 0.1 # Seconds per grid cell motion
    arson_time = 10
    def __init__(self, frame, map, coords):
        self.coords = Math_Utils.Vector(coords[0], coords[1])
        self.frame = frame
        self.map = map
        self.unit_length = map.unit_length
        self.path = []
        self.committing_arson = False
        self.last_time = 0
        self.search = Search(self.map)

    
    def set_fire(self):
        # check adjacent cells for obstacles to burn
        for i in range(-1,2):
            for j in range(-1,2):
                if (abs(i)!=abs(j)):
                    x = self.coords.x + i
                    y = self.coords.y + j
                    if (x > 0 and x < self.map.width and y > 0 and y < self.map.height):
                        if (self.map.occ_grid[y][x] == 1):
                            burned_tet = self.map.referenced_occ_grid[y][x]
                            burned_tet.burning = True
                            self.map.burning_tets.append(burned_tet)
    
    def draw(self):  
        center = Math_Utils.Vector(self.coords.x + self.unit_length/2, self.coords.y + self.unit_length/2)   
        Graphics.fill_rect(self.frame, Colors.orange, center, (self.unit_length,self.unit_length))
        # if len(self.path)>0:
        #     self.search.draw_path(self.frame, self.path, Colors.green, 0.2)

            
    def update(self):
        if (len(self.path) > 0):
            if Clock.time - self.last_time > self.speed :
                self.last_time = Clock.time
                new_cell = self.path.pop(0)
                self.coords = Math_Utils.Vector(new_cell[0], new_cell[1])
        else:
            self.set_fire()
            self.plan_path()
        # print("Skip")

    def plan_path(self):
        start = self.coords.to_tuple()
        
        # Find a target obstacle to go burn
        found = False
        while not found:
            x = random.randint(0,self.map.width-1)
            y = random.randint(0,self.map.height-1)

            for i in range(-1,2):
                for j in range(-1,2):
                    if (abs(i)!=abs(j)):
                        obj_x = x + i
                        obj_y = y + j
                        if (obj_x > 0 and obj_x < self.map.width and obj_y > 0 and obj_y < self.map.height):
                            if (self.map.occ_grid[y][x] == 0):
                                if (self.map.occ_grid[obj_y][obj_x] == 1):
                                    found = True
        
        goal = (x,y)
        print("Goal:")
        print(goal)
        self.path = self.search.Search(2, start, goal, False)
        self.search.draw_path(self.frame, self.path, Colors.green, 0.2)
        # for cell in self.path:
        #     print(cell)

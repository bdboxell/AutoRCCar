from map import *
from valet_car import *
from occupancy_grid import *
import pygame
from wumpus import *
from clock import *
from firetruck import*
import time

'''
    Simulation
        Main class that controls all the flow of events of the simulation
'''
class Simulation:    
    obstacles = []
    iter = 0

    def __init__(self, frame, dimens):
        self.dimens = dimens
        self.frame = frame
        self.draw_background()
        self.sim_start()
        
    def sim_start(self):
        # clock = Clock()
        self.last_real_time = time.time()

        wumpus_start = (125,125)
        self.map = OccupancyGrid(self.frame, 1, 250, 250)
        # block cells for firetruck start
        blocked_cells = [wumpus_start]
        for i in range(0,4):
            for j in range(0, 4):
                cell = (i,j)
                blocked_cells.append(cell)
        self.map.populate(0.05, blocked_cells)


        self.wumpus = Wumpus(self.frame, self.map, wumpus_start)
        self.wumpus.set_fire()

        firetruck_start = Math_Utils.Pose(100,50, math.radians(90))
        self.firetruck = Firetruck(self.frame, firetruck_start)
        self.firetruck.map = self.map

        goal = Math_Utils.Pose(200,200, math.radians(90))
        self.goal_car = Firetruck(self.frame, goal)
        self.goal_car.draw()
        self.map.draw()

        neighbors = self.firetruck.produce_neighbors()
        neighbors = self.firetruck.prune_collisions(neighbors, self.map)

        # neighbor = neighbors[5]
        # neighbor.draw()
        # test_point = Math_Utils.Vector(99,56)
        # Graphics.draw_circle(self.frame, Colors.green, test_point, 0.25)
        # print(neighbor.contains(test_point))

        for car in neighbors:
            car.draw()
        # self.firetruck.plan_path(goal, [])


        

    def draw_background(self):
        Graphics.fill(self.frame, Colors.dirt)

    def update(self):
        self.draw_background()
        self.map.draw()

        self.wumpus.update()
        self.wumpus.draw()

        self.firetruck.update()
        self.firetruck.draw()

        # goal = Math_Utils.Pose(200,50, math.radians(0))
        # self.goal_car = Firetruck(self.frame, goal)
        # self.goal_car.draw()

        # Update simulation timer
        while (time.time() - self.last_real_time < Clock.timestep):
            time.sleep(0.001)
        Clock.increment()
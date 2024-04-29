from map import *
from occupancy_grid import *
import pygame
from clock import *
from car import*
import time

'''
    Simulation
        Main class that controls all the flow of events of the simulation
'''
class Simulation:    

    def __init__(self, frame, dimens):
        self.dimens = dimens
        self.frame = frame
        self.sim_start()
        
    def sim_start(self):
        self.last_real_time = time.time()
        self.map = Map(self.frame,self.dimens, 1,20,(5,15))
        car_start = Pose(50,10, math.radians(0))
        self.car = Car(self.frame, car_start, self.map)

        self.map.init()

    def update(self):
        self.map.draw()
        self.car.update()
        self.car.draw()

        # test_point = Pose(90,38, math.radians(90))
        # Graphics.draw_circle(self.frame, Colors.red, test_point.to_vector(), 1)
        # self.car.plan_path(test_point, [])

        # Update simulation timer
        while (time.time() - self.last_real_time < Clock.timestep):
            time.sleep(0.001)
        Clock.increment()
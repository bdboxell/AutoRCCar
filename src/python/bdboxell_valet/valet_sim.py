from map import *
from valet_car import *

import pygame
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
        
        # Add main car to sim
        pose = Math_Utils.Pose(5,10, math.radians(0))
        self.car = ValetCar(frame, pose) # 2.25, 14 for top left
        
        # Add obstacle cars
        pose = Math_Utils.Pose(8.8,7, math.radians(-30))
        self.obstacles.append(ValetCar(frame, pose))

    def draw_background(self):
        Graphics.fill(self.frame, Colors.gray)

     

    def update(self):
        self.draw_background()
        self.car.draw()
        # for car in self.obstacles:
        #     car.draw()

        

        self.car.update()
        for car in self.obstacles:
            car.update()
            # print(self.car.collides(car))
        
        # point = Math_Utils.Vector(5,4.7)
        # Graphics.draw_circle(self.frame, Colors.green, point, 0.05)
        # print(self.car.contains(point))
            

        

        # self.car.produce_neighbors()
        
    def plan(self):
        target_pose = Math_Utils.Pose(17,8, math.radians(0))
        target = ValetCar(self.frame, target_pose) # 2.25, 14 for top left
        target.draw()
        path = self.car.plan_path(target_pose,[])
        self.car.draw_path(path)
        self.car.path = path
        
    def keyboard_state(self):
        if self.forward_state == 1:
            self.car.drive(100)
        elif self.forward_state == 0:
            self.car.drive(0)
        elif (self.forward_state == -1):
            self.car.drive(-100)

    def key_down(self, key):
        if (key == pygame.K_UP):
            self.forward_state = 1
        elif (key == pygame.K_DOWN):
            self.forward_state = -1
        elif (key == pygame.K_LEFT):
            self.car.steer(100)
        elif (key == pygame.K_RIGHT):
            self.car.steer(-100)
    def key_up(self, key):
        if (key == pygame.K_UP):
            self.forward_state = 0
        if (key == pygame.K_DOWN):
            self.forward_state = 0
        if (key == pygame.K_LEFT):
            self.car.steer(0)
        if (key == pygame.K_RIGHT):
            self.car.steer(0)
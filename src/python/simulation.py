from map import *
from car import *

import pygame
'''
    Simulation
        Main class that controls all the flow of events of the simulation
'''
class Simulation:
    cars = []
    
    forward_state = 0 #0 is neutral, 1 is forward, -1 is braking

    def __init__(self, screen, frame, dimens):
        self.dimens = dimens
        self.track = Map(screen, (0,0),dimens, (1800,1000),20,350,(300,150))
        self.draw_background()
        
        self.car = Car(frame,(200,400))
        self.cars.append(self.car)

    def draw_background(self):
        self.track.draw()
        
    def update(self):
        self.draw_background()
        self.keyboard_state()
        for car in self.cars:
            car.update()
            car.draw()
        
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
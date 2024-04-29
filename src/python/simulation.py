from map import *
from occupancy_grid import *
import pygame
from clock import *
from car import*
import time
from data_logging import *

'''
    Simulation
        Main class that controls all the flow of events of the simulation
'''
class Simulation:    

    def __init__(self, frame, dimens):
        self.dimens = dimens
        self.frame = frame
        self.sim_start()
        self.done = False
        
    def sim_start(self):
        self.last_real_time = time.time()
        self.map = Map(self.frame,self.dimens, 1,20,(5,15))
        car_start = Pose(50,15, math.radians(0))
        self.car = Car(self.frame, car_start, self.map)
        
        car2_start = Pose(50,5, math.radians(0))
        self.car2 = Car(self.frame, car2_start, self.map)
        self.car2.max_speed = 7
        # self.car2.cycle_goal_pose()
        # self.car2.cycle_goal_pose()
        self.car2.color = Colors.red
        self.car.color = Colors.blue
        self.car.car_num = 1
        self.car2.car_num = 2

        self.car.other_cars.append(self.car2)
        self.car2.other_cars.append(self.car)


        self.map.init()

    def update(self):
        if (not self.done and Clock.time > 300):
            Data_Logger.print_data()
            self.done = True
        elif (not self.done):
            self.map.draw()
            self.car2.update()
            # print('Car2 done updating')
            self.car.update()
            # print('Car1 done updating')


            self.car.draw()
            self.car2.draw()

            # test_point = Pose(90,38, math.radians(90))
            # Graphics.draw_circle(self.frame, Colors.red, test_point.to_vector(), 1)
            # self.car2.plan_path(test_point, True)

            # Update simulation timer
            while (time.time() - self.last_real_time < Clock.timestep):
                time.sleep(0.001)
            Clock.increment()
            print(Clock.time)
       
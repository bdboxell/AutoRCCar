from colors import *
from graphics import *
import pygame
from clock import *
import time
from data_logging import *

class Car:
    dimens = (3, 5) # Width x Length
    max_speed = 4
    min_turn_radius = 13
    planning_delta_t = 0.125

    def __init__(self, frame, coords, map):
        self.frame = frame
        self.pose = coords
        self.path = []
        self.speed = 0
        self.wheel_angle = 0
        self.update_self()
        self.last_time = Clock.time
        self.map = map
        self.in_pursuit = False
        self.waypoints = []
        self.color = Colors.green
        self.other_cars = []
        self.goal_poses = [Pose(90,38, math.radians(90)),Pose(50,65, math.radians(-180)),Pose(10,38, math.radians(-90)),Pose(50,10, math.radians(0))]
        self.goal = self.goal_poses[0]
        self.last_update = -999
        self.car_num = 0
    '''
        Start
    '''
    def update_self(self):
        self.direction = Vector(math.cos(self.pose.theta),math.sin(self.pose.theta))
        self.sideways = self.direction.copy()
        self.sideways.rotate(math.radians(90))

        h = self.dimens[0]
        w = self.dimens[1]
        self.corners = []
        self.corners.append(Vector(w/2,h/2))
        self.corners.append(Vector(-w/2,h/2))
        self.corners.append(Vector(-w/2,-h/2))
        self.corners.append(Vector(w/2,-h/2))
        for point in self.corners:
            point.rotate(self.pose.theta)
            point.add(self.pose.to_vector())
            
    '''
        Update
            Called once per loop iteration. Updates all kinematic parameters of the car
    '''
    def update(self):
        # Check to see if you're within sight of the goal
        dist = self.pose.to_vector().distance(self.goal.to_vector())
        #  If there's a path, then follow it
        
        if (Clock.time - self.last_update > 10 or len(self.path)==0):
            if (dist<10):
                self.goal = self.cycle_goal_pose()

            self.path = self.plan_path(self.goal, True)
        else:
            if (self.path[0].timestamp < Clock.time):
                # print("Following path")
                self.in_pursuit = True
                cell = self.path.pop(0)
                self.speed = cell.car.speed
                self.wheel_angle = cell.car.wheel_angle
                self.pose = cell.car.pose
                self.last_pursuit_time = Clock.time
            
        delta_t = Clock.time - self.last_time
        self.last_time = Clock.time
        self.pose = self.kinematics(self.speed, self.wheel_angle, delta_t)

        self.update_self()

    def update_without_planning(self):
        if (len(self.path)>0):
            if (self.path[0].timestamp < Clock.time):
                # print("Following path")
                self.in_pursuit = True
                cell = self.path.pop(0)
                self.speed = cell.car.speed
                self.wheel_angle = cell.car.wheel_angle
                self.pose = cell.car.pose
                self.last_pursuit_time = Clock.time
            
        delta_t = Clock.time - self.last_time
        self.last_time = Clock.time
        self.pose = self.kinematics(self.speed, self.wheel_angle, delta_t)

        self.update_self()

    def cycle_goal_pose(self):
        goal = self.goal_poses.pop(0)
        self.goal_poses.append(goal)
        return goal
    '''
        Draw
            Uses graphics library to draw the car at whatever the current pose is
    '''
    def draw(self):
        Graphics.draw_rect(self.frame, self.color, self.pose.to_vector(), self.dimens, self.pose.theta)
        
        # Uncomment to View Orientation Vectors
        # Graphics.draw_vector(self.frame, Colors.red, self.pose.to_vector(), self.direction)
        # Graphics.draw_vector(self.frame, Colors.green, self.pose.to_vector(), self.sideways)
        # Graphics.draw_circle(self.frame, Colors.black, self.pose.to_vector(), 0.05)
        
        # Uncomment to View collision corners
        # for point in self.corners:
        #     Graphics.draw_circle(self.frame, Colors.yellow, point, 0.25)

    '''
        Collides
            Collision detection for cars

            Returns true if this car is colliding with another car
    '''
    def collides(self, car):
        for point in self.get_corners():
            if (car.contains(point)):
                return True
        for point in car.get_corners():
            if (self.contains(point)):
                return True
        return False

    '''
        Contains
            Used for collision detection
            Returns true if the point exists within the car's footprint
    '''
    def contains(self, p):
        padded_dimens = (self.dimens[0]/2 + 1,self.dimens[1]/2 + 1)
        point = p.copy()
        # Get the point's coordinates relative to the car

        point.subtract(self.pose.to_vector())
        
        # Calculate the projection of the point onto the forward and sideways vectors
        f_point = point.project(self.direction)
        s_point = point.project(self.sideways)

        # DEBUG: uncomment to view the point projected onto the direction vector
        # f_point.add(self.pose.to_vector())
        # Graphics.draw_circle(self.frame, Colors.black, f_point, 0.25)
        # s_point.add(self.pose.to_vector())
        # Graphics.draw_circle(self.frame, Colors.red, s_point, 0.25)
        # f_point.subtract(self.pose.to_vector())
        # s_point.subtract(self.pose.to_vector())

        if (f_point.magnitude() < padded_dimens[1] and s_point.magnitude() < padded_dimens[0]):
            return True
        else:
            return False
        
    def get_corners(self):
        return self.corners
    
    '''
        Kinematics
            A funciton that handles the kinematics of a basic car.
            Given a speed and wheel angle, it predicts where the car will end up in one loop iteration

            Speed: (m/s) speed of the car
            Wheel_Angle: (degrees) wheel angle
            DeltaT: elapsed time
    '''
    def kinematics(self, speed, wheel_angle, deltaT):
        if abs(speed) > self.max_speed:
            speed = speed/abs(speed) * self.max_speed

        length = self.dimens[1]
    
        dist = speed*deltaT

        cur_coords = self.pose.to_vector()
        
        new_pose = None
        if wheel_angle == 0:
            # Handle driving in a straight line
            # direction is already a unit vector that points in the direction of the car
            forward = self.direction.multiply(dist) # parameter is distance in front of car
            # backward = self.direction.multiply(-dist)
            forward.add(cur_coords)
            # backward.add(cur_coords)

            new_pose = Pose(forward.x, forward.y, self.pose.theta)
        else:
            # Calculate ICC
            wheel_angle = math.radians(wheel_angle)

            icc = cur_coords.copy()
            icc.add(self.direction.multiply(-length/2)) # at this point, icc is at the rear axle
            offset = length/math.tan(wheel_angle)
            icc.add(self.sideways.multiply(offset)) # icc coordinates in global frame

            # Find car's coordinates in ICC frame
            coords_icc_frame = cur_coords.copy()
            coords_icc_frame.subtract(icc)
            
            # Rotate them by the amount turned
            radius = length/math.sin(wheel_angle)
            theta = dist/radius
            coords_icc_frame.rotate(theta)

            # Find the new coordinates by adding back in the ICC frame
            forward_pose = coords_icc_frame.copy()
            forward_pose.add(icc)

            new_pose = Pose(forward_pose.x, forward_pose.y, self.pose.theta+theta)

        return new_pose

    '''
        Produce Neighbors
            Produces a list of all new poses that are kinematically possible 
            to reach by the next loop iteration. This funciton is used to 
            generate "neighboring cells" for a classical search algorithm.

            Returns: A list of poses
    '''
    def produce_neighbors(self):
        length = self.dimens[1]

        # start with an empty list
        neighbors = []

        accelerations = [-1, 0, 1]
        # distances = [-1, 1]
        for accel in accelerations: # The distance that the car is capable of driving in the next loop iteration
            for wheel_angle in range(-20,21,5):
                    new_pose = self.kinematics(self.speed+accel, wheel_angle+wheel_angle, self.planning_delta_t)
                    virtual_car = Car(self.frame, new_pose, map)
                    virtual_car.speed = self.speed + accel
                    virtual_car.wheel_angle = wheel_angle
                    neighbors.append(virtual_car)

        # for car in neighbors:
        #     car.draw()
        # pygame.display.flip()
        
        return neighbors
    
    def prune_collisions(self, neighbors, map):
        valid_neighbors = []
        for virtual_car in neighbors:
            # check collision with obstacles
            

            valid = True
            for corner in virtual_car.corners:
                if (not map.point_on_track(corner)):
                    valid = False
                    break
            if valid:
                valid_neighbors.append(virtual_car)
        return valid_neighbors
    
    def causes_accident(self, new_cell):
        valid = True
        for car in self.other_cars:

            t = new_cell.timestamp
            i = 0
            found = False
            replanned = False
            while (not found):
                if (i>=len(car.path)):
                    dist = car.pose.to_vector().distance(car.goal.to_vector())
                    # Graphics.draw_circle(self.frame, Colors.red, car.pose.to_vector(), 1)
                    # print(i * self.planning_delta_t)
                    if (replanned and i * self.planning_delta_t<=t):
                        print('Too far in future to predict')
                        return False
                    elif (Clock.time - car.last_update>5 and not replanned):
                        print('Forcing replan for collision prediction')
                        goal = car.goal
                        car.path = car.plan_path(goal, False)
                        i = 0
                        replanned = True
                    else:
                        return False
                    # Graphics.draw_circle(self.frame, Colors.green, car.goal.to_vector(), 1)
                    

                elif (i>=len(car.path)):
                    # If you've already replanned, and you still can't find it,
                    # it means the algorithm can not see that far into the future.
                        return False
                
                if (abs(car.path[i].timestamp - t)<0.13):
                    found = True
                else:
                    i = i+1
                
            if (new_cell.car.collides(car.path[i].car)):
                # print('potential collision detected')
                valid = False
        return not valid
    '''
        Plan Path
            Handles the path planning for the car

            Uses A* based on the kinematic possibilities found in the
            produce_neighbors function
    '''
    def plan_path(self, target, cognizant):
        start_plan_time = time.time()
        self.last_update = Clock.time
        if (self.speed<=5):
            max_iter = 800
        else:
            max_iter = 200
        
        dangerous_route = False
        iter = 0
        found = False
        # Graphics.draw_circle(self.frame, self.color, self.pose.to_vector(), 1)

        # Graphics.draw_circle(self.frame, Colors.green, target.to_vector(), 1)

        # In this case, the frontier is a list of virtual cars, starting with itself
        cost = self.cost_function(self, target)
        start_cell = Car.Cell(None, self, cost, self.planning_delta_t)
        # return False
        frontier = [start_cell]
        explored = []
        recovery_attempt = 0

        while not found:

            # Sort the frontier based on cost function
            frontier = sorted(frontier, key=lambda x:x.cost)
            cur_cell = frontier.pop(0)
            explored.append(cur_cell)

            cur_cell.car.draw()
            pygame.display.flip()

            neighbors = cur_cell.car.produce_neighbors()
            neighbors = self.prune_collisions(neighbors, self.map)
            for virtual_car in neighbors:

                valid_neighbor = True

                # check to make sure it doesn't already exist
                combined = frontier.copy()
                for cell in explored:
                    combined.append(cell)
                for cell in combined:
                    (dist, delta_theta) = cell.car.pose.distance(virtual_car.pose)
                    if (abs(dist)<0.1 and abs(delta_theta)< math.radians(30)):
                        valid_neighbor = False
                        break
                
                # If it is still a valid neighbor, add it to the frontier
                if valid_neighbor:
                    cost = self.cost_function(virtual_car, target)
                    virtual_car.max_speed = self.max_speed
                    new_cell = Car.Cell(cur_cell, virtual_car, cost, self.planning_delta_t)

                    # Prune for collisisons
                    if (not cognizant):
                        frontier.append(new_cell)
                    elif (cognizant and not self.causes_accident(new_cell)):
                        frontier.append(new_cell)


            
            # Check to see if you've found the target
            (dist, delta_theta) = target.distance(cur_cell.car.pose)
            if (abs(dist)< 8): # and abs(delta_theta) < math.radians(2)):
                found = True
            elif (len(frontier) == 0 or iter>=max_iter):
                # print("Empty Frontier! Backing up...")
                # pose = self.kinematics(-0.5, 5, 0.25)
                # self.pose = pose
                # return self.plan_path(target, cognizant)
            
                # Recovery Modes!
                # First, determine if you are in front of the other cars nearby
                in_front = True
                for car in self.other_cars:
                    car_to_car = car.pose.to_vector()
                    car_to_car.add(self.pose.to_vector().multiply(-1))
                    dist = car_to_car.magnitude()
                    if (dist < 20):
                        unit_car_to_car = car_to_car.project(self.direction).unit_vector()
                        if (unit_car_to_car.equals(self.direction.unit_vector())):
                            in_front = False
                        unit_car_to_car.print()

                # If you are in front, you have right of way. Don't worry about the 
                # other cars.
                if (recovery_attempt >3):
                    "Car #", self.car_num, " in Recovery Mode 1: Ignoring other cars"
                    for i in range(5):
                        self.cycle_goal_pose()
                    target = self.cycle_goal_pose()
                    self.goal = target
                    recovery_attempt = 0
                if (in_front):

                    print("Car #", self.car_num, " in Recovery Mode 1: Ignoring other cars")
                    recovery_attempt = recovery_attempt + 1
                    cognizant = False
                    pause_delta_t = self.planning_delta_t
                    dangerous_route = True
                else:
                    # Otherwise, slow down and let them by.
                    print("Car #", self.car_num, " in Recovery Mode 2: Possibly blocked off, slowing down")
                    recovery_attempt = recovery_attempt + 1
                    pause_delta_t = 3
                    delta_v = 0
                    pose = self.kinematics(0, self.wheel_angle, pause_delta_t)
                    self.pose = pose
                    self.speed = 0
                start_cell = Car.Cell(start_cell, self, 0, pause_delta_t)
                # return False
                frontier = [start_cell]
                explored = []
                iter = 0
                

            iter = iter+1
        
            # # Handle the case where it maxed out iterations
            # if (iter>=max_iter):
            #     print("Possibly blocked off, slowing down")
            #     pause_delta_t = 0.5
            #     delta_v = 0
            #     pose = self.kinematics(0, self.wheel_angle, pause_delta_t)
            #     self.pose = pose
            #     self.speed = self.speed - delta_v
            #     start_cell = Car.Cell(start_cell, self, 0, pause_delta_t)
            #     # return False
            #     frontier = [start_cell]
            #     explored = []
            #     iter = 0
        
        # If you've made it this far, it must have been successful
        # Now backtrack and return the successful path
        

        at_start = False
        path = []
        while not at_start:
            if (cur_cell != None):
                path.append(cur_cell)
                if (cur_cell.parent == None):
                    at_start = True
                else:
                    cur_cell = cur_cell.parent
        path.reverse()
        self.path = path
        if (dangerous_route):
            for car in self.other_cars:
                car.plan_path(car.goal, True)
        duration = time.time() - start_plan_time
        Data_Logger.planning_times.append(duration)
        # print("Goal Found!")
        return path
                

    def cost_function(self, car, target):

        (euc_dist, delta_theta) = target.distance(car.pose)
        if (self.speed<5):
            dist_gain = 30
            wall_dist_gain = -20
            speed_gain = -1
        else:
            dist_gain = 18
            wall_dist_gain = -20
            speed_gain = -1

        min_dist = 9999
        for other_car in self.other_cars:
            car_to_car = other_car.pose.to_vector()
            car_to_car.add(self.pose.to_vector().multiply(-1))
            dist = car_to_car.magnitude()
            if (dist<min_dist):
                min_dist = dist
        # print(min_dist)
        collision_gain = 75/math.pow(min_dist,1)
        # collision_gain = 0


        wall_dist = self.map.dist_from_wall(car.pose.to_vector())

        cost = euc_dist * dist_gain + car.speed*speed_gain + wall_dist*wall_dist_gain + collision_gain

        return cost
    
    def draw_path(self, path):
        for cell in path:
            pose = cell.car.pose
            Graphics.draw_rect(self.frame, Colors.green, pose.to_vector(), self.dimens, pose.theta)

    class Cell:
        def __init__(self, parent, car, cost, delta_t):
            self.cum_cost = 0 if parent==None else parent.cum_cost+5
            self.car = car
            self.cost = cost + self.cum_cost
            self.parent = parent
            self.timestamp = Clock.time if parent==None else parent.timestamp+delta_t

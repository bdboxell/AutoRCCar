from colors import *
from graphics import *
import pygame
from clock import *

class Firetruck:
    dimens = (2.2, 3) # Width x Length
    max_speed = 10
    min_turn_radius = 13
    planning_delta_t = 0.25

    def __init__(self, frame, coords):
        self.frame = frame
        self.pose = coords
        self.path = []
        self.speed = 0
        self.wheel_angle = 0
        self.update_self()
        self.last_time = Clock.time
        self.map = None
        self.in_pursuit = False
    '''
        Start
    '''
    def update_self(self):
        self.direction = Math_Utils.Vector(math.cos(self.pose.theta),math.sin(self.pose.theta))
        self.sideways = self.direction.copy()
        self.sideways.rotate(math.radians(90))

        h = self.dimens[0]
        w = self.dimens[1]
        self.corners = []
        self.corners.append(Math_Utils.Vector(w/2,h/2))
        self.corners.append(Math_Utils.Vector(-w/2,h/2))
        self.corners.append(Math_Utils.Vector(-w/2,-h/2))
        self.corners.append(Math_Utils.Vector(w/2,-h/2))
        for point in self.corners:
            point.rotate(self.pose.theta)
            point.add(self.pose.to_vector())
            
    '''
        Update
            Called once per loop iteration. Updates all kinematic parameters of the car
    '''
    def update(self):
        if (len(self.path) > 0):
            if (self.path[0].timestamp < Clock.time):
                print("Following path")
                self.in_pursuit = True
                cell = self.path.pop(0)
                self.speed = cell.car.speed
                self.wheel_angle = cell.car.wheel_angle
                self.pose = cell.car.pose
                self.last_pursuit_time = Clock.time
        elif (self.in_pursuit):
            print("In Position!")
            self.speed = 0
            self.wheel_angle = 0
            if (Clock.time - self.last_pursuit_time > 5):
                self.target_tet.burning = False
                self.in_pursuit = False

        else:
            self.speed = 0
            self.wheel_angle = 0
            if (len(self.map.burning_tets) > 0):
                self.target_tet = self.map.burning_tets.pop(0)
                goal = Math_Utils.Pose(self.target_tet.coords[0]*self.map.unit_length,self.target_tet.coords[1]*self.map.unit_length,0)
                self.path = self.plan_path(goal, [])
                print(len(self.path))

        delta_t = Clock.time - self.last_time
        self.last_time = Clock.time
        self.pose = self.kinematics(self.speed, self.wheel_angle, delta_t)

        self.update_self()

    '''
        Draw
            Uses graphics library to draw the car at whatever the current pose is
    '''
    def draw(self):
        Graphics.draw_rect(self.frame, Colors.blue, self.pose.to_vector(), self.dimens, self.pose.theta)
        
        # Graphics.draw_vector(self.frame, Colors.red, self.pose.to_vector(), self.direction)
        # Graphics.draw_vector(self.frame, Colors.green, self.pose.to_vector(), self.sideways)
        # Graphics.draw_circle(self.frame, Colors.black, self.pose.to_vector(), 0.05)
        
        # for point in self.corners:
        #     Graphics.draw_circle(self.frame, Colors.yellow, point, 0.25)


    '''
        Collides
            Collision detection for cars

            Returns true if this car is colliding with another car
    '''
    def collides(self, car):
        print("Self:")
        for point in self.get_corners():
            point.print()
            if (car.contains(point)):
                return True
        print("Other:")
        for point in car.get_corners():
            point.print()
            if (self.contains(point)):
                return True
        return False

    '''
        Contains
            Used for collision detection
            Returns true if the point exists within the car's footprint
    '''
    def contains(self, p):
        padded_dimens = (self.dimens[0]/2 + 0.5,self.dimens[1]/2 + 0.5)
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

            new_pose = Math_Utils.Pose(forward.x, forward.y, self.pose.theta)
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

            new_pose = Math_Utils.Pose(forward_pose.x, forward_pose.y, self.pose.theta+theta)

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
            for wheel_angle in range(-15,16,5):
                    new_pose = self.kinematics(self.speed+accel, wheel_angle+wheel_angle, self.planning_delta_t)
                    virtual_car = Firetruck(self.frame, new_pose)
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
            collision_radius = math.floor(math.sqrt(math.pow(self.dimens[0]/2,2) + math.pow(self.dimens[1]/2,2)))
            nearest_cell = (math.floor(virtual_car.pose.x), math.floor(virtual_car.pose.y))

            valid = True
            if (virtual_car.pose.x < 0 or virtual_car.pose.x > map.width):
                valid = False
            elif (virtual_car.pose.y < 0 or virtual_car.pose.y > map.width):
                valid = False
            else:
                for i in range (-collision_radius, collision_radius+1):
                    x = nearest_cell[0] + i
                    for j in range (-collision_radius, collision_radius+1):
                        y = nearest_cell[1] + j
                        if (x>=0 and x<map.width and y>=0 and y<map.height):
                        
                            if (map.occ_grid[y][x] == 1):
                                
                                corners = [(x,y),(x+map.unit_length,y),(x,y+map.unit_length),(x+map.unit_length,y+map.unit_length)]
                                for corner in corners:
                                    corner = Math_Utils.Vector(corner[0],corner[1])

                                    if (virtual_car.contains(corner)):
                                        valid = False
                                        break
            if valid:
                valid_neighbors.append(virtual_car)
        return valid_neighbors
    '''
        Plan Path
            Handles the path planning for the car

            Uses A* based on the kinematic possibilities found in the
            produce_neighbors function
    '''
    def plan_path(self, target, obstacles):
        max_iter = 100000
        iter = 0
        found = False

        # In this case, the frontier is a list of virtual cars, starting with itself
        cost = self.cost_function(self, target)
        start_cell = Firetruck.Cell(None, self, cost, self.planning_delta_t)
        # return False
        frontier = [start_cell]
        explored = []

        while not found and iter<max_iter:

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

                # THIS COLLISION DETECTION IS BROKEN BECAUSE OF BREAK STATEMENT
                # # check collision with obstacles
                # for obstacle in obstacles:
                #     if virtual_car.collides(obstacle):
                #         valid_neighbor = False
                #         break

                # check to make sure it doesn't already exist
                combined = frontier.copy()
                for cell in explored:
                    combined.append(cell)
                for cell in combined:
                    (dist, delta_theta) = cell.car.pose.distance(virtual_car.pose)
                    if (abs(dist)<0.2 and abs(delta_theta)< math.radians(30)):
                        valid_neighbor = False
                        break
                
                # If it is still a valid neighbor, add it to the frontier
                if valid_neighbor:
                    cost = self.cost_function(virtual_car, target)
                    new_cell = Firetruck.Cell(cur_cell, virtual_car, cost, self.planning_delta_t)
                    frontier.append(new_cell)
            
            # Check to see if you've found the target
            (dist, delta_theta) = target.distance(cur_cell.car.pose)
            if (abs(dist)< 10): # and abs(delta_theta) < math.radians(2)):
                found = True
            elif (len(frontier) == 0):
                print("Empty Frontier!")
                return False

            iter = iter+1
        
        # Handle the case where it maxed out iterations
        if (iter>=max_iter):
            print("Unable to find goal: Maxed Iterations")
            return False
        
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

        print("Goal Found!")
        return path
                

    def cost_function(self, car, target):

        (euc_dist, delta_theta) = target.distance(car.pose)

        # Within 50 meters, start to penalize for driving fast
        slow_threshhold = 40
        if (euc_dist > slow_threshhold):
            speed_gain = -2
        else:
            speed_gain = 0.001

        backward_gain = 0
        if (car.speed<0):
            backward_gain = 0.5

        cost = euc_dist * 25 + car.speed*speed_gain + car.speed*backward_gain

        # print(cost)
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

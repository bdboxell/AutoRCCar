from colors import *
from graphics import *
import pygame
import time

class ValetCar:
    dimens = (4.5, 2)

    def __init__(self, frame, coords):
        self.frame = frame
        self.pose = coords
        self.path = None
        self.speed = 0
        self.wheel_angle = 0
        self.update()
        self.last_time = time.time()

    '''
        Update
            Called once per loop iteration. Updates all kinematic parameters of the car
    '''
    def update(self):
        if not self.path == None:
            if (len(self.path)>0):
                cell = self.path.pop(0)
                self.pose = cell.car.pose

        self.direction = Math_Utils.Vector(math.cos(self.pose.theta),math.sin(self.pose.theta))
        self.sideways = self.direction.copy()
        self.sideways.rotate(math.radians(90))

        w = self.dimens[0]
        h = self.dimens[1]
        self.corners = []
        self.corners.append(Math_Utils.Vector(w/2,h/2))
        self.corners.append(Math_Utils.Vector(-w/2,h/2))
        self.corners.append(Math_Utils.Vector(-w/2,-h/2))
        self.corners.append(Math_Utils.Vector(w/2,-h/2))
        
        for point in self.corners:
            point.rotate(self.pose.theta)
            point.add(self.pose.to_vector())

    '''
        Draw
            Uses graphics library to draw the car at whatever the current pose is
    '''
    def draw(self):
        Graphics.draw_rect(self.frame, Colors.blue, self.pose.to_vector(), self.dimens, self.pose.theta)
        
        Graphics.draw_vector(self.frame, Colors.red, self.pose.to_vector(), self.direction)
        Graphics.draw_vector(self.frame, Colors.green, self.pose.to_vector(), self.sideways)
        Graphics.draw_circle(self.frame, Colors.black, self.pose.to_vector(), 0.05)
        
        for point in self.corners:
            Graphics.draw_circle(self.frame, Colors.yellow, point, 0.1)


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
        point = p.copy()
        # Get the point's coordinates relative to the car

        point.subtract(self.pose.to_vector())
        
        # Calculate the projection of the point onto the forward and sideways vectors
        f_point = point.project(self.direction)
        s_point = point.project(self.sideways)

        # DEBUG: uncomment to view the point projected onto the direction vector
        f_point.add(self.pose.to_vector())
        Graphics.draw_circle(self.frame, Colors.black, f_point, 0.05)
        s_point.add(self.pose.to_vector())
        Graphics.draw_circle(self.frame, Colors.black, s_point, 0.05)
        f_point.subtract(self.pose.to_vector())
        s_point.subtract(self.pose.to_vector())

        if (f_point.magnitude() < self.dimens[0]/2 and s_point.magnitude() < self.dimens[1]/2):
            return True
        else:
            return False
        
    def get_corners(self):
        return self.corners
    
    def kinematics(self, speed, wheel_angle):
        length = self.dimens[0]
        elapsed_time = time.time() - self.last_time
        dist = speed*elapsed_time

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

            new_pose = Math_Utils.Pose(new_pose.x, new_pose.y, self.pose.theta+theta)

        return new_pose
    
    '''
        Produce Neighbors
            Produces a list of all new poses that are kinematically possible 
            to reach by the next loop iteration. This funciton is used to 
            generate "neighboring cells" for a classical search algorithm.

            Returns: A list of poses
    '''
    def produce_neighbors(self):
        length = self.dimens[0]

        # start with an empty list
        neighbors = []

        changes_in_speed = [-1]
        # distances = [-1, 1]
        for dist in distances: # The distance that the car is capable of driving in the next loop iteration
            for wheel_angle in range(-60,70,15):
            
                    neighbors.append(Math_Utils.Pose(new_pose.x, new_pose.y, self.pose.theta+theta))

        # for pose in neighbors:
        #     Graphics.draw_rect(self.frame, Colors.red, pose.to_vector(), self.dimens, pose.theta)
        # pygame.display.flip()
        
        return neighbors
    '''
        Plan Path
            Handles the path planning for the car

            Uses A* based on the kinematic possibilities found in the
            produce_neighbors function
    '''
    def plan_path(self, target, obstacles):
        max_iter = 10000
        iter = 0
        found = False

        # In this case, the frontier is a list of virtual cars, starting with itself
        cost = self.cost_function(self, target)
        start_cell = ValetCar.Cell(None, self, cost)
        # return False
        frontier = [start_cell]
        explored = []

        while not found and iter<max_iter:

            # Sort the frontier based on cost function
            frontier = sorted(frontier, key=lambda x:x.cost)
            cur_cell = frontier.pop(0)
            explored.append(cur_cell)

            pose = cur_cell.car.pose
            Graphics.fill(self.frame, Colors.gray)
            Graphics.draw_rect(self.frame, Colors.blue, pose.to_vector(), self.dimens, pose.theta)
            # for cell in frontier:
            #     pose = cell.car.pose
            #     Graphics.draw_rect(self.frame, Colors.green, pose.to_vector(), self.dimens, pose.theta)
            pygame.display.flip()

            neighbors = cur_cell.car.produce_neighbors()
            for neighbor in neighbors:
                # create a virtual car out of the neighbor
                virtual_car = ValetCar(self.frame, neighbor)

                # check collision with obstacles
                valid_neighbor = True
                for obstacle in obstacles:
                    if virtual_car.collides(obstacle):
                        valid_neighbor = False
                        break

                # check to make sure it doesn't already exist
                combined = frontier.copy()
                for cell in explored:
                    combined.append(cell)
                for cell in combined:
                    (dist, delta_theta) = cell.car.pose.distance(neighbor)
                    if (abs(dist)<0.1 and abs(delta_theta)< math.radians(3)):
                        valid_neighbor = False
                        break
                
                # If it is still a valid neighbor, add it to the frontier
                if valid_neighbor:
                    cost = self.cost_function(virtual_car, target)
                    new_cell = ValetCar.Cell(cur_cell, virtual_car, cost)
                    frontier.append(new_cell)
            
            # Check to see if you've found the target
            (dist, delta_theta) = target.distance(cur_cell.car.pose)
            if (abs(dist)< 0.1 and abs(delta_theta) < math.radians(2)):
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
            path.append(cur_cell)
            cur_cell = cur_cell.parent
            if (cur_cell.parent == None):
                path.append(cur_cell)
                at_start = True
        path.reverse()
        
        print("Goal Found!")
        return path
                

    def cost_function(self, car, target):

        (euc_dist, delta_theta) = target.distance(car.pose)
        cost = euc_dist
        # if (euc_dist > 5):
        #     target.subtract(car.pose)
        #     optimal_theta = math.atan2(target.y, target.x)
        #     heading_error = car.pose.theta - optimal_theta
        #     cost = euc_dist*5 + 5*heading_error
        # else:
        #     # Prioritize neighbors with a lower off-axis error
        #     print("Cost Function")
        #     # Express target in car's frame
        #     target.subtract(car.pose)
        #     target = target.to_vector()
        #     sideways = target.project(car.sideways)
        #     forward = target.project(car.direction)

        #     s_point = sideways.copy()
        #     s_point.add(car.pose.to_vector())
        #     f_point = forward.copy()
        #     f_point.add(car.pose.to_vector())
        #     Graphics.draw_circle(self.frame, Colors.black, s_point, 0.05)
        #     Graphics.draw_circle(self.frame, Colors.black, f_point, 0.05)

        #     side_error = sideways.magnitude()
        #     forward_error = forward.magnitude()
        #     # print(side_error)
        #     # print(forward_error)
        #     cost = 3*sideways.magnitude() + forward.magnitude()
        #     # if (euc_dist < 1):
        #     #     cost = cost + 5*abs(delta_theta)
        #     # forward = t_in_car.project(car.direction).magnitude()
        #     # (dist, delta_theta) = target.distance(car.pose)
        #     # cost = 10*sideways + forward
        #     # if abs(dist < 0.5) and abs(delta_theta) > math.radians(10):
        #     #     cost = 999999
        print(cost)
        return cost
    
    def draw_path(self, path):
        for cell in path:
            pose = cell.car.pose
            Graphics.draw_rect(self.frame, Colors.green, pose.to_vector(), self.dimens, pose.theta)



    class Cell:
        def __init__(self, parent, car, cost):
            self.cum_cost = 0 if parent==None else parent.cum_cost+1
            self.car = car
            self.cost = cost + self.cum_cost
            self.parent = parent
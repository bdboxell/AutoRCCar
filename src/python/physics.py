from math_utils import *
import time
from graphics import *
from colors import *

'''
    Physics

    The physics class contains classes that handle the math for a general physics engine
'''
class Physics:

    '''
        Force

        Class used to represent and handle all the math regarding summing forces
    '''
    class Force:
        magnitude = None
        direction = None
        force_vector = None
        '''
            Constructor

            Magnitude: Scalar magnitude of the force vector
            Direction: (Vector) unit direction vector
                Note: if direction vector is not a unit vector, it will be automatically corrected
        '''
        def __init__(self, magnitude, direction):
            self.magnitude = magnitude

            # Scale direction to a unit vector
            direction = direction.unit_vector()
            self.direction = direction

            # Calculate force vector
            self.force_vector = direction.multiply(magnitude)

        '''
            Add

            Adds another force vector to this force
        '''
        def add(self, force):
            self.force_vector.x = self.force_vector.x + force.force_vector.x
            self.force_vector.y = self.force_vector.y + force.force_vector.y

        '''
            Update Magnitude Direction

            Updates the magnitude and direction based on the force vector. It is important to keep those syncronized
        '''
        def update_magnitude_direction(self):
            self.magnitude = self.force_vector.magnitude()
            self.direction = self.force_vector.unit_vector()

    '''
        Force

        Class used to represent a dampener and handles the math converting a dampener to a force
    '''
    class Dampener:
        direction = None
        max = 0
        '''
            Constructor

            Direction: (Vector) unit direction vector
                Note: if direction vector is not a unit vector, it will be automatically corrected
            Constant: Dampening constant
            Max: Max force the dampener is capable of exerting
        '''
        def __init__(self, direction, constant, max):
            direction = direction.unit_vector()
            self.direction = direction
            self.constant = constant
            self.max = max

        '''
            Force Convert
                Calculates the force on the body from the dampener based on the velocity
            
            Velocity: (Vector) Velocity vector of the body
        '''

        def force_convert(self, velocity):
            velocity_component = velocity.project(self.direction)

            magnitude = velocity_component.magnitude()*self.constant
            if (not velocity_component.unit_vector().equals(self.direction)):
                self.direction = self.direction.multiply(-1)
    
            force = Physics.Force(magnitude, self.direction)
            
            return force
        
    '''
        Kinematics
            Class used to handle all equations of motion of a body
    '''
    class Kinematics:
        forces = []
        dampeners = []
        pose = None

        
        def __init__(self, frame, pose, mass):
            self.mass = mass
            self.pose = pose
            self.velocity = Math_Utils.Vector(0,0)
            self.acceleration = Math_Utils.Vector(0,0)
            self.last_timestamp = time.time()
            self.frame = frame

        def update(self):
            elapsed_time = time.time() - self.last_timestamp
            if (elapsed_time != 0):
                # Sum all the forces that were applied to the car in this frame
                net_force = Physics.Force(0,Math_Utils.Vector(0,0))
                for dampener in self.dampeners:
                    adj_velocity = self.velocity.copy()
                    adj_velocity.rotate(-self.pose.theta)
                    self.forces.append(dampener.force_convert(adj_velocity))
               
                for force in self.forces:
                    net_force.add(force)

                net_force.force_vector.rotate(self.pose.theta)
                net_force.update_magnitude_direction()
                
                self.forces.clear()
                self.dampeners.clear()
                # Graphics.draw_vector(self.frame, Colors.red, (self.pose.x, self.pose.y), net_force)

                # Acceleration is the sum of net forces divided by the car's mass
                self.acceleration = net_force.force_vector.multiply(1/self.mass)
                # self.acceleration.theta = self.acceleration.theta*self.mass/self.MOI
                # Graphics.draw_vector(self.frame, Colors.red, (self.pose.x, self.pose.y), self.acceleration)

                # Increase velocity based on acceleration
                self.velocity.add(self.acceleration.multiply(elapsed_time))
                # if abs(self.acceleration.theta) <1:
                #     self.velocity.theta = 0

                # Increase position based on velocity
                delta = self.velocity.multiply(elapsed_time)
                delta = Math_Utils.Pose(delta.x, delta.y, 0)
                self.pose.add(delta)

                self.last_timestamp = time.time()

                return self.pose
from colors import *
from math_utils import *
import math
from graphics import *
from physics import *

class Car:
    pose = Math_Utils.Pose(0,0,0)
    forces = []
    drag_coeff = -6#kg*m/s
    max_str = 30 #degrees
    mass = 2.5 #kg
    friction_coeff = 1
    side_bite = -10
    gravity = 9.8
    max_wheel_force = 2000
    MOI = 10000

    def __init__(self, frame, coords):
        self.coords = coords
        self.dimens = (5,100) # width x length
        self.pose = Math_Utils.Pose(coords[0],coords[1],0) # x, y, yaw
       
        self.wheel_angle = 0
        self.frame = frame
        self.cur_throttle = 0
        self.kinematics = Physics.Kinematics(self.frame, self.pose, self.mass)

    def draw(self):
        center = (self.pose.x, self.pose.y)
        Graphics.draw_rect(self.frame, Colors.car, center, self.dimens, self.pose.theta)
        Graphics.draw_circle(self.frame, Colors.black, center, 5)
    
    def update(self):
        self.add_gas()
        self.add_drag()
        self.add_side_bite()
        self.pose = self.kinematics.update()    
    
    def add_drag(self):
        if (self.cur_throttle == 0):
            direction = Math_Utils.Vector(1,0)
            max = 9999999999
            drag_brake = Physics.Dampener(direction, self.drag_coeff, max)
            self.kinematics.dampeners.append(drag_brake)
    
    def add_side_bite(self):
        direction = Math_Utils.Vector(0,1)
        max = 9999999999
        side_bite = Physics.Dampener(direction, self.side_bite, max)
        self.kinematics.dampeners.append(side_bite)

    # def add_steering(self):
    #     if (self.wheel_angle!=0):
    #         # Max force that friction can supply to the wheels
    #         max_force = self.mass * self.gravity * self.friction_coeff

    #         # Calculate the turning center
    #         print(self.wheel_angle)
    #         offset = self.dimens[1]/math.tan(math.radians(self.wheel_angle))
    #         radius = self.dimens[1]/math.sin(math.radians(self.wheel_angle))
    #         turn_center = Math_Utils.Pose(-self.dimens[1]/2, offset,0)
    #         # turn_center.print()

    #         # Calculate the centripetal force vector resulting from turning
    #         centripetal = math.pow(self.velocity.magnitude(),2)*self.mass/radius
    #         front_wheel_center = Math_Utils.Pose(self.dimens[1]/2, 0,0)
    #         turn_copy = turn_center.copy()
    #         wheel_copy = front_wheel_center.copy()
    #         # self.pose.print()
    #         turn_copy.transform(self.pose)
    #         wheel_copy.transform(self.pose)
    #         # turn_copy.print()
    #         Graphics.draw_circle(self.frame, Colors.black, (turn_copy.x,turn_copy.y), 5)
    #         Graphics.draw_circle(self.frame, Colors.red, (wheel_copy.x, wheel_copy.y), 5)

    #         turn_center.transform(self.pose)
    #         front_wheel_center.transform(self.pose)
            
    #         turn_center.subtract(front_wheel_center)
    #         unit_force_vector = turn_center.multiply(1/radius)
            
    #         # Compare to friction
    #         # real_centripetal = min(max_force, centripetal)
    #         force_vector = unit_force_vector.multiply(centripetal)
    #         # Graphics.draw_vector(self.frame, Colors.red, (self.pose.x, self.pose.y), force_vector)
    #         # force_vector.print()
            
    #         # Calculate the torque applied to the robot
    #         alpha = math.atan2(force_vector.y, force_vector.x)
    #         phi = alpha - self.pose.theta
    #         torque = force_vector.magnitude()*math.sin(phi)*self.dimens[1] /2
    #         force_vector.theta = torque

    #         # force_vector.transform(self.pose)
            
    #         # Debug
    #         # force_vector.x = 0
    #         # force_vector.y = 0
    #         # force_vector.theta = 0
    #         Graphics.draw_vector(self.frame, Colors.red, (self.pose.x, self.pose.y), force_vector)
    #         self.forces.append(force_vector)           

    # def add_friction(self):
    #     sideways_vector = Math_Utils.Pose(math.cos(math.pi/2-self.pose.theta), math.sin(math.pi/2-self.pose.theta),0) 
    #     sideways_velocity = self.velocity.project(sideways_vector).magnitude()
    #     friction_vector = sideways_vector.multiply(sideways_velocity * self.side_bite)
    #     Graphics.draw_vector(self.frame, Colors.green, (self.pose.x, self.pose.y), friction_vector)

        
    #     self.forces.append(friction_vector)


    def steer(self, pct):
        angle = pct*self.max_str/100
        # self.wheel_angle = angle
        self.kinematics.pose.theta = self.kinematics.pose.theta - math.pi/2


    def add_gas(self):
        F = (self.cur_throttle/100)*self.max_wheel_force
        force = Physics.Force(F, Math_Utils.Vector(1,0))
        self.kinematics.forces.append(force)

    def drive(self, F):
        self.cur_throttle = F

           
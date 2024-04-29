import pygame
from colors import *
from math_utils import *
from graphics import *
'''
    Map
        The class responsible for all information and actions regarding the track
'''
class Map:
    '''
        Constructor
            dimens: Vector width and height of the bounding box of the track
            lane_width: width of the lanes of the track
            corner_radii: (Tuple) radius of the (inner corners, outer corners)
    '''
    def __init__(self, frame,dimens, pipe_width, lane_width, corner_radii):
        self.frame = frame
        self.dimens = dimens
        self.pipe_width = pipe_width
        self.lane_width = lane_width
        self.corner_radii = (corner_radii[0], lane_width+corner_radii[0])

    def init(self):
        center = Vector(self.dimens.x/2, self.dimens.y/2)

        width = self.dimens.x - 2*self.lane_width - 2*self.corner_radii[0]
        height = self.dimens.y - 2*self.lane_width - 2*self.corner_radii[0]

        self.infield_box = Rectangle(Vector(1,1), Vector(1,1)) #Dummy values, gets overwritten in next line
        self.infield_box.def_by_center(center, Vector(width, height))

    '''
        Point On Track
            Returns true if the specified point is on the track. Used for 
            collision detection with edges. Works by finding the nearest 
            x and y coordinates on the infield box, then calculates distance
            from that point. If the distance is within the bounds of the track,
            it is on the track.
    '''
    def point_on_track(self, point):
        # Find x coordinate on infield box
        if (point.x < self.infield_box.top_left.x):
            x = self.infield_box.top_left.x
        elif (point.x > self.infield_box.top_right.x):
            x = self.infield_box.top_right.x 
        else:
            x = point.x

        # Find y coordinate on infield box
        if (point.y > self.infield_box.top_left.y):
            y = self.infield_box.top_left.y
        elif (point.y < self.infield_box.bottom_left.y):
            y = self.infield_box.bottom_right.y 
        else:
            y = point.y

        ref_point = Vector(x,y)
        distance = point.distance(ref_point)

        if (distance > self.corner_radii[0] and distance < self.corner_radii[0] + self.lane_width):
            return True
        else:
            return False

    def dist_from_wall(self, point):
        # Find x coordinate on infield box
        if (point.x < self.infield_box.top_left.x):
            x = self.infield_box.top_left.x
        elif (point.x > self.infield_box.top_right.x):
            x = self.infield_box.top_right.x 
        else:
            x = point.x

        # Find y coordinate on infield box
        if (point.y > self.infield_box.top_left.y):
            y = self.infield_box.top_left.y
        elif (point.y < self.infield_box.bottom_left.y):
            y = self.infield_box.bottom_right.y 
        else:
            y = point.y

        ref_point = Vector(x,y)
        distance = point.distance(ref_point)
        inner_dist = distance - self.corner_radii[0]
        outer_dist = self.corner_radii[0] + self.lane_width - distance

        if (inner_dist < outer_dist):
            return inner_dist
        else:
            return outer_dist
    '''
        Draw
            Handles drawing the track

    '''
    def draw(self):
        Graphics.fill(self.frame, Colors.grass)
        pipe_vector = Vector(self.pipe_width,self.pipe_width).multiply(2)
        outer_dimens = self.dimens.copy()
        outer_dimens.add(pipe_vector)
        Graphics.fill_rounded_rect(self.frame, Colors.pipe, self.infield_box.center(), outer_dimens.to_tuple(), self.corner_radii[1]+self.pipe_width)
        Graphics.fill_rounded_rect(self.frame, Colors.dirt, self.infield_box.center(), self.dimens.to_tuple(), self.corner_radii[1])
        inner_radius_vector = Vector(self.corner_radii[0],self.corner_radii[0]).multiply(2)
        inner_pipe_dimens = self.infield_box.dimens.copy()
        inner_pipe_dimens.add(inner_radius_vector)
        Graphics.fill_rounded_rect(self.frame, Colors.pipe, self.infield_box.center(), inner_pipe_dimens.to_tuple(), self.corner_radii[0])
        pipe_vector = Vector(self.pipe_width,self.pipe_width).multiply(-2)
        inner_pipe_dimens.add(pipe_vector)
        Graphics.fill_rounded_rect(self.frame, Colors.grass, self.infield_box.center(), inner_pipe_dimens.to_tuple(), self.corner_radii[0]-self.pipe_width)

import pygame
import math
from math_utils import *

'''
    Graphics
        Class responsible for drawing all pygame graphics in the correct frame
'''
class Graphics:
    screen = None
    ppm = None # Pixels Per Meter
    def __init__(self, screen):
        self.screen = screen

    '''
        Draw Rect
            Draws a rectangle in the specified frame

            Color: Color of the rectangle
            Frame: Reference frame to be drawn in
            Center: (Vector) Coordinates of the center of the rectangle
            Dimens: (Tuple) Width x Length
            Theta: Angle of the rectangle
    '''
    def draw_rect(frame, color, center, dimens, theta):
        w = dimens[0]
        h = dimens[1]
        points = []
        points.append(Math_Utils.Vector(w/2,h/2))
        points.append(Math_Utils.Vector(-w/2,h/2))
        points.append(Math_Utils.Vector(-w/2,-h/2))
        points.append(Math_Utils.Vector(w/2,-h/2))
        
        for point in points:
            point.rotate(theta)
            point.add(center)

        
        for i in range(4):
            j = i+1
            if j == 4:
                j = 0
            
            
            Graphics.draw_line(frame, color, points[i], points[j], 0.1)

        # # Determine the points that define the rectangle
        # start = (center[0] - length*math.cos(theta)/2, center[1] - length*math.sin(theta)/2)
        # end = (center[0] + length*math.cos(theta)/2, center[1] + length*math.sin(theta)/2)

        # Graphics.draw_line(frame, color, start, end, width)

    '''
        Draw Line
            Draws a line in the specified frame

            Color: Color of the line
            Frame: Reference frame to be drawn in
            Start: (Tuple) Coordinates of line start
            End: (Tuple) Coordinates of line end
            Width: Line thickness

    '''
    def draw_line(frame, color, start, end, width):
        start = Graphics.transform_coords(frame, start.to_tuple())
        end = Graphics.transform_coords(frame, end.to_tuple())
        width = int(width*Graphics.ppm)
        pygame.draw.line(Graphics.screen, color, start, end, width)
    
    '''
        Draw Circle
            Draws a circle in the specified frame

            Color: Color of the circle
            Frame: Reference frame to be drawn in
            Coords: (Tuple) Coordinates of the center
            Radius: Radius of the circle
    '''
    def draw_circle(frame, color, coords, radius):
        # Apply frame transform        # Apply frame transform
        radius = radius*Graphics.ppm
        coords = Graphics.transform_coords(frame,coords.to_tuple())
        pygame.draw.circle(Graphics.screen, color,coords, radius)

    '''
        Transform Coords
            Transforms frame coordinates to pygame coordinates. Uses pixels per meter
    '''
    def transform_coords(frame, coords):
        coords = (int(coords[0]*Graphics.ppm+frame.x),int(frame.y - coords[1]*Graphics.ppm))
        return coords
    
    '''
        Draw Vector
            Draws a vector from a point
        
            Color: Color of the vector
            Frame: Reference frame to be drawn in
            Start: (Vector) Start Point
            Vector: (Vector) Vector to be drawn
    '''
    
    def draw_vector(frame, color, start, vector):
        end = vector.copy()
        end.add(start)
        Graphics.draw_line(frame, color, start, end, 0.05)

    '''
        Fill
            Fills the frame with a solid color
    '''

    def fill(frame, color):
        center = (frame.x + frame.width/2, frame.y+frame.height/2)
        dimens = (frame.width, frame.height)
        # Graphics.draw_rect(frame, color, center, dimens, 0)
        Graphics.screen.fill(color)

    '''
        Frame
            Defines a frame within the screen to draw in
    '''
    class Frame:
        def __init__(self, coords, dimens):
            self.coords = coords
            self.dimens = dimens
            self.x = coords.x
            self.y = coords.y
            self.width = dimens.x
            self.height = dimens.y
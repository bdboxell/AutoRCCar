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
        h = dimens[0]
        w = dimens[1]
        points = []
        points.append(Vector(w/2,h/2))
        points.append(Vector(-w/2,h/2))
        points.append(Vector(-w/2,-h/2))
        points.append(Vector(w/2,-h/2))
        
        for point in points:
            point.rotate(theta)
            point.add(center)

        
        for i in range(4):
            j = i+1
            if j == 4:
                j = 0
            
            
            Graphics.draw_line(frame, color, points[i], points[j], 0.25)

    '''
        Fill Rect
            Draws a rectangle in the specified frame

            Color: Color of the rectangle
            Frame: Reference frame to be drawn in
            Center: (Vector) Coordinates of the center of the rectangle
            Dimens: (Tuple) Width x Length
    '''
    def fill_rect(frame, color, center, dimens):
        w = dimens[0]
        h = dimens[1]
        top_left = Vector(-w/2,h/2)
        top_left.add(center)
        
        top_left = Graphics.transform_coords(frame, top_left.to_tuple())
        pygame.draw.rect(Graphics.screen, color, (top_left[0], top_left[1], w*Graphics.ppm,h*Graphics.ppm))

    '''
        Fill Rounded Rect
            Draws a filleted rectangle in the specified frame

            Color: Color of the rectangle
            Frame: Reference frame to be drawn in
            Center: (Vector) Coordinates of the center of the rectangle
            Dimens: (Tuple) Width x Length
            R: Corner radius
    '''
    def fill_rounded_rect(frame, color, center, dimens, r):
        w = dimens[0]
        h = dimens[1]
        base_width = w - 2*r
        base_height = h - 2*r

        base_rect = Rectangle(Vector(1,1), Vector(1,1))
        base_rect.def_by_center(center, Vector(base_width,base_height))
        Graphics.fill_rect(frame, color, center, (base_width, h))
        Graphics.fill_rect(frame, color, center, (w, base_height))
        Graphics.draw_circle(frame, color, base_rect.top_left, r)
        Graphics.draw_circle(frame, color, base_rect.top_right, r)
        Graphics.draw_circle(frame, color, base_rect.bottom_left, r)
        Graphics.draw_circle(frame, color, base_rect.bottom_right, r)


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
            Coords: (Vector) Coordinates of the center
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
        Graphics.draw_line(frame, color, start, end, 0.1)

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
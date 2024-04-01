import pygame
import sys
import math
import random

black = (0,0,0)
dark_blue = (75, 112, 171)

'''
    Tetromino
        The representation of one tetromino that makes up walls
'''
class Tetromino:
    '''
    Constructor
        Screen: Reference to pygame screen
        Unit Length: Side length of one square
        Type: (Drawn in orienntation 0 deg)
            1: ▯▯▯▯
            2: ▯▯
                ▯▯
            3:  ▯▯
               ▯▯
            4: ▯▯▯
                 ▯
            5: ▯▯▯
               ▯
            6:
               ▯▯
               ▯▯
        Orientation: Tetromino Orientation (degrees, 0 is along positive x axis)
        Coords: tuple, coordinates of the upper left most unit square
    '''
    def __init__(self, screen, unit_length, type, orientation, coords, color):
        self.type = type
        self.orientation = math.radians(-orientation)
        self.coords = coords
        self.tetromino_style = []
        self.occupied_cells = []
        self.color = color
        self.border_color = dark_blue
        self.border_width = 1
        self.unit_length = unit_length
        self.screen = screen
        self.populate_occupancy()

    '''
        Populate Occupancy
            Creates the tetromino based on type, orientation, and coordinates
    '''
    def populate_occupancy(self):
        if (self.type == 1):
            self.tet_1()
        elif (self.type == 2):
            self.tet_2()
        elif (self.type == 3):
            self.tet_3()
        elif (self.type == 4):
            self.tet_4()
        elif (self.type == 5):
            self.tet_5()
        elif (self.type == 6):
            self.tet_6()
        else:
            print("Invalid Tetromino Type")


        for cell in self.tetromino_style:
            rot_coords = self.rotate_coordinates(cell, self.orientation)
            trans_coords = (int(rot_coords[0] + self.coords[0]), int(rot_coords[1] + self.coords[1]))
            self.occupied_cells.append(trans_coords)
        
    '''
        Rotate Coordinates
            Transforms coordinates by a rotation about the origin by the specified angle

            Coords: coordinate pair to be transformed
            Theta: angle to rotate it by
    '''
    def rotate_coordinates(self,coords, theta):
        new_x = round(coords[0]*math.cos(theta) - coords[1]*math.sin(theta))
        new_y = round(coords[0]*math.sin(theta) + coords[1]*math.cos(theta))
        return new_x,new_y

    '''
    Draw Tetromino
        Handles drawing one tetromino

        tet: the tetromino to draw
    '''
    def draw(self):
        for cell in self.occupied_cells:
            coord = ((self.unit_length * cell[0]),(self.unit_length * cell[1]))
            pygame.draw.rect(self.screen, self.color, (coord[0], coord[1], self.unit_length, self.unit_length))
            top, right, bottom, left = True, True, True, True
            for adj_cell in self.occupied_cells:
                if (cell[0] == adj_cell[0] and cell[1]-1 == adj_cell[1]):
                    top = False
                elif (cell[0] == adj_cell[0] and cell[1]+1 == adj_cell[1]):
                    bottom = False
                elif (cell[0]+1 == adj_cell[0] and cell[1] == adj_cell[1]):
                    right = False
                elif (cell[0]-1 == adj_cell[0] and cell[1] == adj_cell[1]):
                    left = False
            if (top):
                pygame.draw.line(self.screen, self.border_color, (coord[0],coord[1]), (coord[0]+self.unit_length,coord[1]), self.border_width)
            if (bottom):
                pygame.draw.line(self.screen, self.border_color, (coord[0],coord[1]+self.unit_length), (coord[0]+self.unit_length,coord[1]+self.unit_length), self.border_width)
            if (right):
                pygame.draw.line(self.screen, self.border_color, (coord[0]+self.unit_length,coord[1]), (coord[0]+self.unit_length,coord[1]+self.unit_length), self.border_width)
            if (left):
                pygame.draw.line(self.screen, self.border_color, (coord[0],coord[1]), (coord[0],coord[1]+self.unit_length), self.border_width)


    '''
        Tetromino Constructors

        Each shape of tetromino is manually defined here.
        Refer to the class constructor description for a guide to each shape.
    '''
    def tet_1(self):
        self.tetromino_style.append((0,0))
        self.tetromino_style.append((1,0))
        self.tetromino_style.append((2,0))
        self.tetromino_style.append((3,0))
    
    def tet_2(self):
        self.tetromino_style.append((0,0))
        self.tetromino_style.append((1,0))
        self.tetromino_style.append((1,1))
        self.tetromino_style.append((2,1))
    
    def tet_3(self):
        self.tetromino_style.append((0,0))
        self.tetromino_style.append((1,0))
        self.tetromino_style.append((0,1))
        self.tetromino_style.append((-1,1))
    
    def tet_4(self):
        self.tetromino_style.append((0,0))
        self.tetromino_style.append((1,0))
        self.tetromino_style.append((2,0))
        self.tetromino_style.append((2,1))

    def tet_5(self):
        self.tetromino_style.append((0,0))
        self.tetromino_style.append((1,0))
        self.tetromino_style.append((2,0))
        self.tetromino_style.append((0,1))

    def tet_6(self):
        self.tetromino_style.append((0,0))
        self.tetromino_style.append((1,0))
        self.tetromino_style.append((0,1))
        self.tetromino_style.append((1,1))



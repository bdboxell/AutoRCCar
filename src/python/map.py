import pygame
from colors import *
'''
    Map
        The class responsible for all information and actions regarding the track
'''
class Map:
    '''
        Constructor
            coords: (Tuple) (x,y) position of the top left corner of the bounding box of the track
            dimens: (Tuple) width and height of the bounding box of the track
            lane_width: width of the lanes of the track
            corner_radii: (Tuple) radius of the (inner corners, outer corners)
    '''
    def __init__(self, screen, coords, frame_dimens, track_dimens, pipe_width, lane_width, corner_radii):
        self.screen = screen
        self.coords = coords
        self.frame_dimens = frame_dimens
        self.track_dimens = track_dimens
        self.pipe_width = pipe_width
        self.lane_width = lane_width
        self.corner_radii = corner_radii

    '''
        Draw
            Handles drawing the track

    '''
    def draw(self):
        pygame.draw
        pygame.draw.rect(self.screen, Colors.grass, (self.coords[0],self.coords[1], self.frame_dimens[0], self.frame_dimens[1]))
        center = (self.coords[0]+self.frame_dimens[0]/2, self.coords[1]+self.frame_dimens[1]/2)
        bounding_box = pygame.Rect(center[0] - self.track_dimens[0]/2, center[1] - self.track_dimens[1]/2, self.track_dimens[0], self.track_dimens[1])
        pipe_box = pygame.Rect(center[0] - self.track_dimens[0]/2 - self.pipe_width, center[1] - self.track_dimens[1]/2 - self.pipe_width, self.track_dimens[0]+2*self.pipe_width, self.track_dimens[1]+2*self.pipe_width)
        pygame.draw.rect(self.screen, Colors.pipe, pipe_box, 0, self.corner_radii[0]+self.pipe_width)
        pygame.draw.rect(self.screen, Colors.dirt, bounding_box, 0, self.corner_radii[0])

        inner_pipe_box = pygame.Rect(center[0] - self.track_dimens[0]/2 + self.lane_width, center[1] - self.track_dimens[1]/2 + self.lane_width, self.track_dimens[0] - 2*self.lane_width, self.track_dimens[1] - 2*self.lane_width)        
        inner_bounding_box = pygame.Rect(center[0] - self.track_dimens[0]/2 + self.lane_width + self.pipe_width, center[1] - self.track_dimens[1]/2 + self.lane_width + self.pipe_width, self.track_dimens[0] - 2*self.lane_width - 2*self.pipe_width, self.track_dimens[1] - 2*self.lane_width - 2*self.pipe_width)        
        
        pygame.draw.rect(self.screen, Colors.pipe, inner_pipe_box, 0, self.corner_radii[1])
        pygame.draw.rect(self.screen, Colors.grass, inner_bounding_box, 0, self.corner_radii[1] - self.pipe_width)

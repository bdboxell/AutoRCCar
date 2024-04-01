import pygame
import sys
from simulation import *
from graphics import *
from math_utils import *

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 1920,1080
dimens = (width, height)
screen = pygame.display.set_mode(dimens)
pygame.display.set_caption("Motion Planner")

Graphics.screen = screen
sim_frame = Math_Utils.Pose(0,1080,0)
simulation = Simulation(screen, sim_frame,dimens)


# Main pygame loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        elif event.type == pygame.KEYDOWN:
            simulation.key_down(event.key)
        elif event.type == pygame.KEYUP:
            simulation.key_up(event.key)   

    simulation.update()


    # Update display
    pygame.display.flip()

    # Control the frame rate
    pygame.time.Clock().tick(60)


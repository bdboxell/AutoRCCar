import pygame
import sys
from valet_sim import *
from graphics import *
from math_utils import *

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 20,15
pixels_per_meter = 60
dimens = (width*pixels_per_meter, height*pixels_per_meter)
screen = pygame.display.set_mode(dimens)
pygame.display.set_caption("Motion Planner")

Graphics.screen = screen
Graphics.ppm = pixels_per_meter
sim_frame = Graphics.Frame(Math_Utils.Vector(0,dimens[1]),Math_Utils.Vector(dimens[0],dimens[1]))
simulation = Simulation(sim_frame,dimens)

simulation.plan()

# Main pygame loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        # elif event.type == pygame.KEYDOWN:
        #     simulation.key_down(event.key)
        # elif event.type == pygame.KEYUP:
        #     simulation.key_up(event.key)   

    simulation.update()


    # Update display
    pygame.display.flip()

    # Control the frame rate
    pygame.time.Clock().tick(60)


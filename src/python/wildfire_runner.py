import pygame
import sys
from wildfire_sim import *
from graphics import *
from math_utils import *

# Initialize Pygame
pygame.init()

# Set up the display
width, height = 250,250
pixels_per_meter = 4
dimens = (width*pixels_per_meter, height*pixels_per_meter)
print(dimens)
screen = pygame.display.set_mode(dimens)
pygame.display.set_caption("Motion Planner")

Graphics.screen = screen
Graphics.ppm = pixels_per_meter

sim_frame = Graphics.Frame(Math_Utils.Vector(0,dimens[1]),Math_Utils.Vector(dimens[0],dimens[1]))
simulation = Simulation(sim_frame,dimens)


# Main pygame loop
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    simulation.update()

    # map.draw()
    # Update display
    pygame.display.flip()

    # Control the frame rate
    pygame.time.Clock().tick(60)


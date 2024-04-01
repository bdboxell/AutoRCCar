from occupancy_grid import *
import pygame
import time
red = (255,0,0)
light_green = (138, 235, 180)
dark_green = (0, 105, 45)
yellow = (255,255,0)
'''
    Search
        Class that contains all code related to search and path generation
'''
class Search:
    def __init__(self, occ_grid):
        self.occ_grid = occ_grid

    '''
        Search
            Performs a search to navigate between points on the map. This one method handles BFS, DFS, and Dijkstra's through a parameter.

            Type: The parameter that determines the type of search being done
                0: BFS
                1: DFS
                2: Dijkstra's
                3: Random
            Start: (Tuple) Start coordinates
            Goal: (Tuple) End coordinates
            Animate: (Boolean) If the search animation should play
    '''
    def Search(self, type, start, goal, animate):
        max_iter = 30000
        iter = 0
        found = False
        start_cell = Cell(start[0],start[1], None, 0)
        frontier = [start_cell]

        search_grid = []
        for row in self.occ_grid.occ_grid:
            search_grid.append(list(row))
        search_grid[start[1]][start[0]] = -1

        # print("Planning Route...")
        while not found and iter<max_iter:
            if type==0:
                cur_cell = frontier.pop(0)
            elif type==1:
                cur_cell = frontier.pop(len(frontier)-1)
            elif type == 2:
                frontier = sorted(frontier, key=lambda x: x.cost)
                cur_cell = frontier.pop(0)
            else:
                cur_cell = frontier.pop(random.randint(0,len(frontier)-1))
            search_grid[cur_cell.y][cur_cell.x] = -1
            neighbors = self.get_neighbors(cur_cell)
            for cell in neighbors:
                if (search_grid[cell.y][cell.x] == 0):
                    frontier.append(cell)
                    search_grid[cell.y][cell.x] = -2
            if (cur_cell.x == goal[0] and cur_cell.y == goal[1]):
                found = True
            elif (len(frontier) == 0):
                # print("Unable to reach goal: Empty Frontier")
                return False
            iter = iter+1

            if animate:
                self.occ_grid.draw_cell_type(search_grid, -1, light_green)
                self.occ_grid.draw_cell_type(search_grid, -2, dark_green)
                pygame.display.flip()

        if (iter>=max_iter):
            print("Unable to find goal: Maxed Iterations")
            return False
        
        at_start = False
        path = []
        while not at_start:
            path.append((cur_cell.x,cur_cell.y))
            cur_cell = cur_cell.parent
            if (cur_cell.parent == None):
                path.append((cur_cell.x,cur_cell.y))
                at_start = True
        path.reverse()
        # print("Goal Found!")
        print(type, ", ", iter)
        return path

    '''
        Draw Path
            Draws the path on the pygame UI

            Path: (List of tuples) List of coordinates that represent the path
            Color: (Tuple) The color to draw the line
            Width: Line width
    '''
    def draw_path(self, path, color, width):
        for i in range(0,len(path)-1):
            start = (path[i][0]*self.occ_grid.unit_length + self.occ_grid.unit_length/2, path[i][1]*self.occ_grid.unit_length + self.occ_grid.unit_length/2)
            end = (path[i+1][0]*self.occ_grid.unit_length + self.occ_grid.unit_length/2, path[i+1][1]*self.occ_grid.unit_length + self.occ_grid.unit_length/2)
           
            pygame.draw.line(self.occ_grid.screen, color, start, end, width)

    '''
        Get Neighbors
            Returns a list of coordinates that are the neighboring cells to a specified cell
        
        Cur_Cell: Starting coordinate
    '''
    def get_neighbors(self, cur_cell):
        neighbors = []
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                cell = Cell(cur_cell.x + i, cur_cell.y + j, cur_cell, cur_cell.cost + 1)
                if (not(i==0 and j==0) and abs(i)!=abs(j)):
                    if (cell.x >= 0 and cell.x < self.occ_grid.width):
                        if (cell.y >= 0 and cell.y < self.occ_grid.height):
                            neighbors.append(cell)
        return neighbors
    
class Cell:
    def __init__(self, x,y, parent, cost):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost
    
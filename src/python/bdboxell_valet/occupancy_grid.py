from tetromino import *


light_blue = (133, 192, 214)

'''
    OccupancyGrid
        The representation of the map
'''
class OccupancyGrid:
    '''
    Constructor
        Screen: Reference to pygame screen
        Unit Length: Side length of one square
        Width: Width of occupancy grid
        Height: Height of occupancy grid
    '''
    def __init__(self, screen, unit_length, width, height):
        self.screen = screen
        self.unit_length = unit_length
        self.width = width
        self.height = height
        self.occ_grid = [[0 for _ in range(width)] for _ in range(height)]
        self.tet_list = []

    '''
        Populate_Tetrominos
            Creates an occupancy grid filled to a desired density

            Fill Percent: % of occupancy grid that is wall
            Empty Cells: A list of cells that are guaranteed to be empty after population
    ''' 
    def populate(self, fill_percent, empty_cells):
        num_tets = math.floor(self.width*self.height*fill_percent/4)
        
        for i in range(num_tets):
            valid = False
            while not valid:
                type = random.randint(1,6)
                orientation = random.randint(0,3)*90
                coords = (random.randint(0,self.width-1),random.randint(0,self.height-1))
                tet = Tetromino(self.screen, self.unit_length, type, orientation, coords, light_blue)
                valid = True
                for cell in tet.occupied_cells:
                    if (cell[0] < 0 or cell[0] >= self.width):
                        valid = False
                    elif (cell[1] < 0 or cell[1] >= self.height):
                        valid = False
                    elif (self.occ_grid[cell[1]][cell[0]] == 1):
                        valid = False
                    for empty_cell in empty_cells:
                        if (cell[0] == empty_cell[0] and cell[1] == empty_cell[1]):
                            valid = False
                if valid:
                    self.tet_list.append(tet)
                    for cell in tet.occupied_cells:
                        self.occ_grid[cell[1]][cell[0]] = 1

    '''
        Draw
            Draws all the tetrominos with borders
    '''
    def draw(self):
        for tet in self.tet_list:
            tet.draw()

    '''
        Draw Occupancy Grid
            Draws the representation of the occupancy grid generated
    '''
    def draw_basic(self):
        for x in range(0,self.width):
            for y in range(0,self.height):
                if (self.occ_grid[y][x] == 1):
                    coord = ((self.unit_length * x),(self.unit_length * y))
                    pygame.draw.rect(self.screen, black, (coord[0], coord[1], self.unit_length, self.unit_length))

    '''
        Draw Cell Type
            Draws cells of the specified type at the specified color

            Type: Number
            Color: Cell Color
    '''
    def draw_cell_type(self, grid, type, color):
        for x in range(0,self.width):
            for y in range(0,self.height):
                if (grid[y][x] == type):
                    coord = ((self.unit_length * x),(self.unit_length * y))
                    pygame.draw.rect(self.screen, color, (coord[0], coord[1], self.unit_length, self.unit_length))

    '''
        Count Occupanccy Grid

        Prints the number of occupied cells out of the total number
    '''
    def count_occ_grid(self):
        count = 0
        for x in range(0,self.width):
            for y in range(0,self.height):
                if (self.occ_grid[y][x] == 1):
                    count = count+1
        print("\n", count, " cells are occupied out of ", self.width*self.height)
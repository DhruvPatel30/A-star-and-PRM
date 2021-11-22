

import pygame
import math
import random
from collections import deque
import time
from robot import *
import threading
import global_variables as g_v
from A_star import  global_AStar, AStar
from PRM import ProbabilisticRoadMap
import numpy as np

WIDTH = 1000
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("Wild Fire")

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

total_cells = 250*250                                             # 250*250
obstacle_cells = 0                                              # as we are creating random obstacles for specified density we need to keep track.

# To built pygame environment.
class Spot:                                                         
    def __init__(self, row, col, width, total_rows):
        self.row = row 
        self.col = col 
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.X = (0,0)
        self.trace = []

    def is_barrier(self):
        return self.color == GREEN
    
    def is_green_or_red(self):
        return (self.color == GREEN or self.color == RED)

    def make_closed(self):
        self.color = RED
    
    def make_highlight(self):
        self.color = BLACK

    def make_barrier(self):
        self.color = GREEN
    
    def make_car(self):
        self.color = BLUE
    
    def clear_car(self):
        self.color = WHITE

    def clear_trail(self):
        self.color = WHITE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x , self.y, self.width, self.width))

    def __lt__(self, other):
        return False

#####   FUNCTION TO DRAW GRIDS IN PYGAME #####
def make_grid(rows, width):
    grid = []
    gap = width // rows
    for i in range(rows):
        grid.append([])
        for j in range(rows):
            spot = Spot(i, j, gap, rows)
            grid[i].append(spot)
    
    return grid

def draw_grid(win, row, width):
    gap = width / row
    for i in range(row):
        pygame.draw.line(win, GREY, (0 , i*gap), (width, i*gap))
        for j in range(row):
            pygame.draw.line(win, GREY, (j*gap, 0), (j*gap, width))

def draw(win, grid, rows, width): 
    win.fill(WHITE)
    for row in grid:
        for spot in row:
            spot.draw(win)
    
    # draw_grid(win, rows, width)
    pygame.display.update()

##### END OF FUNCTION TO DRAW GRIDS IN PYGAME #####

class Patch:
    def __init__(self,grid, patch_size):
        self.path_size = patch_size
        self.total_patch_size = patch_size**2
        self.track = 0
        self.overall_track = 0
        self.percent_to_filled = 70
        self.grid_track = []                                        # tracks the grid on obstacle in the first 15*15 grid(make_patch). Which is then shifted.
        self.final_grid_track = []
        self.track_patch = []
        self.obstacle_location = []
        self.list_of_individual_obstacle = []
        self.one_patch = []                                          # keeps the track of the individual patch. (56)
        self.near_patch_track = []                                   # keeps the track of the nearby patch. Which needs to be fired.
        self.track_fire = 0                                          # tracks the no. of cells that are lit.
        self.store = []
        g_v.DISTANCE = [[float('inf') for col in range(250)] for row in range(250)]
        g_v.PREVIOUS = [[(-10, -10, -10) for col in range(250)] for row in range(250)]

        g_v.G_DISTANCE = [[float('inf') for col in range(250)] for row in range(250)]
        g_v.G_PREVIOUS = [[(-10, -10, -10) for col in range(250)] for row in range(250)]

        self.queue = deque()
        self.burnt_track = []
        self.total_burn = 0
        self.total_extinguish = 0
        self.extinguish_fire = False
        self.total_time = 0


    def tetrominoes_obstacle_1(self, grid, i,j):                      # Obstacle type 1.
        self.grid_track.append([(i,j), (i,j+1), (i,j+2), (i,j+3)])

    def tetrominoes_obstacle_2(self, grid, i,j):                      # Obstacle type 2.
        self.grid_track.append([(i,j), (i+1,j), (i+1,j+1), (i+1,j+2)])

    def tetrominoes_obstacle_3(self, grid, i,j):                      # # Obstacle type 3.
        self.grid_track.append([(i,j), (i,j+1), (i+1,j+1), (i+1,j+2)])

    def make_patch(self, grid):
        while self.track < self.total_patch_size*self.percent_to_filled/100:
            x = random.randint(0, 12)
            y = random.randint(0,12)
            obstacle = random.randint(1,3)

            if obstacle == 1:
                if grid[x][y].is_barrier != True:
                    if grid[x+1][y].is_barrier != True and grid[x+2][y].is_barrier != True and grid[x+3][y].is_barrier != True:
                        self.tetrominoes_obstacle_1(grid, x,y)
                        self.track += 4

            elif obstacle == 2:
                if grid[x][y].is_barrier != True:
                    if grid[x][y+1].is_barrier != True and grid[x+1][y+1].is_barrier != True and grid[x+1][y+2].is_barrier != True:
                        self.tetrominoes_obstacle_2(grid, x,y)
                        self.track += 4

            elif obstacle == 3:
                if grid[x][y].is_barrier != True:
                    if grid[x+1][y].is_barrier != True and grid[x+1][y+1].is_barrier != True and grid[x+2][y+1].is_barrier != True:
                        self.tetrominoes_obstacle_3(grid, x,y)
                        self.track += 4

        self.overall_track += 225

    def shift_patch(self, grid):
        x_shift = random.randint(0, 15)*15
        y_shift = random.randint(0, 15)*15
        
        self.obstacle_location = []  
        for j, shift in enumerate(self.grid_track):
            individual_obstacle = []
            for i, (x, y) in enumerate(shift):
                new_x = x + x_shift
                new_y = y + y_shift
                if i==0 and j==0:
                    self.track_patch.append([new_x,new_y])                  # one point in each patch.To keep track of the patch.                
                individual_obstacle.append((new_x, new_y))
                self.list_of_individual_obstacle.append([new_x, new_y])
                grid[new_x][new_y].make_barrier()
            # self.track_patch.append([x,y])
            # obstacle_location.append(individual_obstacle)
            self.obstacle_location.append(individual_obstacle)

        self.one_patch.append(self.obstacle_location)

    def euclidiean_distance(self,loc1, loc2):
        return math.sqrt((loc1[0] - loc2[0])**2 + (loc1[1] - loc2[1])**2)

    def lit_one_obstacle_and_find_nearPatches(self):
        print("One obstacle lit up. It will spread the fire after 20 seconds !!!")
        
        while True:
            i = random.randint(0, len(self.track_patch)-1)
            if i not in self.store:
                self.store.append(i)
                self.X = self.track_patch[i]
                break

        grid[self.X[0]][self.X[1]].make_closed()
        self.queue.append([self.X[0], self.X[1]])
        self.burnt_track.append([self.X[0], self.X[1]])
        self.total_burn += 1
        self.extinguish_fire = False

        for j, point in enumerate(self.track_patch):
            if (self.euclidiean_distance(self.X, point) <= 30*math.sqrt(2)):
                self.near_patch_track.append(j)
                # grid[point[0]][point[1]].make_path()
        # draw(win, grid, ROWS, width)
        # print(self.total_burn)    
        threading.Timer(5, self.lit_one_obstacle_and_find_nearPatches).start()

    def spread_fire(self):

        if self.extinguish_fire == False:
            print("Spreading fire !!")
            for i in self.near_patch_track:
                for j in self.one_patch[i]:
                    for k in j:
                        # print(len(j))
                        if self.euclidiean_distance(k, self.X) <= 30:
                            grid[k[0]][k[1]].make_closed()
                            self.burnt_track.append([k[0], k[1]])
                            self.total_burn += 1
                            # self.track_fire +=1
                        # print(self.track_fire)
        # draw(win, grid, ROWS, width)
        threading.Timer(10, self.spread_fire).start()

    def give_obstacle_info(self):
        return self.list_of_individual_obstacle

ROWS = 250
width = WIDTH                        
grid = make_grid(ROWS,width)  
win = WIN

def main(win, width):                                      # Main loop
    percentage_to_be_filled = 20                          # percentage of the total grids that needs to be filled with obstcale.
    global obstacle_cells
    
    ##### PORTION TO CREATE AND SHIFT THE GREEN PATCH IN THE ENVIRONMENT #####
    patch = Patch(grid, 15)
    patch.make_patch(grid)
    while(obstacle_cells < total_cells*percentage_to_be_filled/100):
        # patch.make_patch(grid)
        patch.shift_patch(grid)
        obstacle_cells += patch.overall_track
    ##### PORTION TO CREATE AND SHIFT THE GREEN PATCH IN THE ENVIRONMENT #####

    # print("pppp",len(patch.list_of_individual_obstacle))

    patch.lit_one_obstacle_and_find_nearPatches()    # To lit one obstacle and then threading has been used.
    
    threading.Timer(10, patch.spread_fire).start()   # To spread the fire after 20 seconds.

    robot = spawn_robot(grid)           # to spawn the robot at random location and orientation.
    # print(patch.total_burn)

    t_start = time.time()
    prm = ProbabilisticRoadMap()
    t_end_ = time.time()
    patch.total_time += (t_end_-t_start)
    prm.sample(patch, grid)
    print("start")
    t_end = time.time() + 60 * 3
    while time.time() < t_end:
        r_x = robot.location[0]
        r_y = robot.location[1]
        r_h = robot.orientation
        start = r_x, r_y, r_h
        start_ = np.array([r_x, r_y])
        start_1= [r_x, r_y]
        # print("start", start_)
        prm.find_n_near_neighbors(start_)
        prm.join_vertices(start_1, grid)
        try:
            g_v.global_goal = patch.burnt_track[0]
        except:
            continue
        
        prm.find_n_near_neighbors(g_v.global_goal)                     # for goal as the edge can intersect eith obstcale, neighors can be devreased
        prm.join_vertices(g_v.global_goal, grid, flag=True)
        # prm.visualize_edges(grid)
        for point in prm.list_of_n_neighbors:
            prm.edges_d[(str(point[0])+','+str(point[1]))].append(g_v.global_goal)
        # prm.visualize_edges(grid)
        # draw(win, grid, ROWS, width)
        t_start = time.time()
        result = global_AStar(grid, robot, start_1, g_v.global_goal, patch, prm)
        t_end_ = time.time()
        patch.total_time += (t_end_-t_start)
        for way_points in result:
            r_x = robot.location[0]
            r_y = robot.location[1]
            r_h = robot.orientation
            start = r_x, r_y, r_h
            # print(way_points)
            t_start = time.time()
            AStar(grid, robot, start, way_points, patch)
            t_end_ = time.time()
            patch.total_time += (t_end_-t_start)           
            g_v.DISTANCE = [[float('inf') for col in range(250)] for row in range(250)]
            g_v.PREVIOUS = [[(-10, -10, -10) for col in range(250)] for row in range(250)]
        
        g_v.G_DISTANCE = [[float('inf') for col in range(250)] for row in range(250)]
        g_v.G_PREVIOUS = [[(-10, -10, -10) for col in range(250)] for row in range(250)]
        draw(win, grid, ROWS, width)
    
    print("Total time", patch.total_time)
    print("Total obstacles", len(patch.list_of_individual_obstacle))
    print("Total burn", patch.total_burn)
    print("Total extinguish", patch.total_extinguish)
    print("Total intact", len(patch.list_of_individual_obstacle) - patch.total_burn)
    pygame.quit()

    return




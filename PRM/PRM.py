# from environment import *
from time import sleep

import pygame
from numpy.compat import py3k
import environment
import random
import global_variables as g_v
import numpy as np
from scipy.spatial import KDTree, cKDTree

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.adj = {}
        self.edge = {}

class ProbabilisticRoadMap:
    def __init__(self):
        # self.obstacles = obstacles
        self.sampled_points = []             # Vertices
        self.edges = []                      # Edges
        self.total_sample_points = g_v.total_sample_points
        self.ndarray_of_smapled_points = np.zeros(shape= (self.total_sample_points, 2))
        self.kd_tree = cKDTree(self.ndarray_of_smapled_points, leafsize=25)
        self.n_neighbors = g_v.n_neighbors
        self.list_of_n_neighbors = []
        self.edges_d = {}

    def sample(self, patch, grid):
        sample_point = 0
        while sample_point < self.total_sample_points:
            x = random.randint(1, 249)
            y = random.randint(1, 249)
            flag = False
            point = [x,y]
            for obstacle in patch.give_obstacle_info():
                if point == obstacle:
                    flag = True
                    break
            if flag == False:
                self.sampled_points.append(point)
                sample_point += 1

        # self.draw_sample_point(grid)
        self.convert_list_to_ndarray()
        self.generate_kdTree()
        self.create_roadmap(grid)
        
        return

    def draw_sample_point(self, grid):
        for point in self.sampled_points:
            # print(point)
            grid[point[0]][point[1]].color = environment.ORANGE

    def convert_list_to_ndarray(self):
        self.ndarray_of_smapled_points = np.array(self.sampled_points)
        # print(len(self.ndarray_of_smapled_points))

    def generate_kdTree(self):
        self.kd_tree = cKDTree(self.ndarray_of_smapled_points, leafsize=25)
        # self.kd_tree = KDTree(self.ndarray_of_smapled_points, leafsize=25)
    
    def find_n_near_neighbors(self, point):
        dist, ind = self.kd_tree.query(point, k = self.n_neighbors)            # ind will be the incides of the sampled_points. From that we can get the nearest points. 
        self.find_coordinated_of_neighbor(ind)

    def find_coordinated_of_neighbor(self,ind):
        self.list_of_n_neighbors = []
        for i in range(1, len(ind)):
            # print(self.sampled_points[ind[i]])
            self.list_of_n_neighbors.append(self.sampled_points[ind[i]])


    def make_bresenham_lines(self, point, neighbor, grid, flag=False):
        x, y = point
        x2, y2 = neighbor
        line = []
        temp = x,y
        line.append(temp)
        dx = abs(x2 - x)
        dy = -abs(y2 - y)
        sx = 1 if x < x2 else -1
        sy = 1 if y < y2 else -1
        err = dx + dy
        while (x != x2 or y != y2):
            e2 = 2*err
            if (e2 >= dy):
                err += dy
                x += sx
            if (e2 <= dx):
                err += dx
                y+=sy
            temp = x, y

            if flag == True:                                # to visualize grid map, pass flag = True in visualise_edges()
                grid[x][y].color = environment.BLACK

            line.append(temp)
        temp = x2, y2
        line.append(temp)
        return line

    def check_if_possible_edge(self, list_of_points, grid):
        for each_point in list_of_points:
            if grid[each_point[0]][each_point[1]].is_green_or_red():
                return False
        return True

    def create_roadmap(self, grid):                                   # remember to clear self.list_of_n_neighbor (DOne in find_n_neighbors)
        for point in self.sampled_points:
            self.find_n_near_neighbors(point)
            self.join_vertices(point, grid)

        # print(self.edges)
        # self.visualize_edges(grid)

    def join_vertices(self, point, grid, flag=False):
        possible_edges_from_one_point = []
        pos_edge = []
        for neighbor in self.list_of_n_neighbors:
            each_point_of_line =self.make_bresenham_lines(point, neighbor, grid)
            if self.check_if_possible_edge(each_point_of_line, grid) or flag == True:
                temp = [point, neighbor]
                pos_edge.append(neighbor)
                possible_edges_from_one_point.append(temp)    

        self.edges_d[str(point[0])+','+str(point[1])] = pos_edge
        self.edges.append(possible_edges_from_one_point)

    def visualize_edges(self, grid):
        for edge_list in self.edges:
            for edge_points in edge_list:
                point1 = edge_points[0]
                point2 = edge_points[1]
                self.make_bresenham_lines(point1, point2, grid, flag=True)

                
            
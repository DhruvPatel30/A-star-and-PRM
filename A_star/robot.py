import math
import random
import environment
import A_star
import global_variables as g_v


def spawn_robot(grid):
    robot = Robot(g_v.wid_rob, g_v.len_rob, g_v.wheelbase_rob, g_v.min_turn_radius_rob, g_v.max_vel_rob, g_v.start_location, g_v.start_orientation)

    while(True):      
        x = random.randint(2,248)
        y = random.randint(2,248)
        orientation = random.randint(0, 360)
        orientation = orientation * math.pi/ 180
        robot.location = x,y
        robot.orientation = orientation
        robot.make_robot()
        if (robot.check_collision(grid) == False):
            robot.draw_robot(grid)
            break
    
    return robot

class Robot:
    def __init__(self, width, length, wheelbase, min_turn_radius, max_vel, location, orientation):
        self.width = width
        self.length = length
        self.wheelbase = wheelbase
        self.min_turn_radius = min_turn_radius
        self.max_vel = max_vel
        self.location = location
        # self.x = self.location[0]
        # self.y = self.location[1]
        self.orientation = orientation
        self.robot_corner_points = [[3,1], [3,-1], [-1,-1], [-1,1]]
        self.robot_lines = []
        self.rasterized_lines = []
        self.trace = []

    def make_corner_point_pairs(self):          # This function will make pair of 2 consecutive corner points. So that we can draw Bresenham line between 2 points and draw our robot.
        self.robot_lines = []             # Store the corner points of robot in either clockwise or anti clockwise direction    
        for j in range(len(self.robot_corner_points)):
            line = []
            line.append(self.robot_corner_points[j])
            if (j == len(self.robot_corner_points) - 1):
                line.append(self.robot_corner_points[0])
            else:
                line.append(self.robot_corner_points[j + 1])
            self.robot_lines.append(line)

    def make_bresenham_lines(self):
        self.rasterized_lines = []
        for i in self.robot_lines:
            x, y = i[0]
            x2, y2 = i[1]
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
                line.append(temp)
            temp = x2, y2
            line.append(temp)
            self.rasterized_lines.append(line)


    def make_robot(self):
        points =[[3,1], [3,-1], [-1,-1], [-1,1]]
        for i, point in enumerate(points):
            new_point_x = point[0]*math.cos(self.orientation) - point[1]*math.sin(self.orientation)
            new_point_y = point[0]*math.sin(self.orientation) + point[1]*math.cos(self.orientation)
            new_point_x = int(round(new_point_x))
            new_point_y = int(round(new_point_y))
            self.robot_corner_points[i][0] = self.location[0] + new_point_x
            self.robot_corner_points[i][1] = self.location[1] + new_point_y
            # print(new_point_x, new_point_y)

            if self.robot_corner_points[i][0] <=2 or self.robot_corner_points[i][0] >=245 or self.robot_corner_points[i][1]<=2 or self.robot_corner_points[i][1] >=247:   # boundary check
                # print("jjjjjjjjjjjjj")
                return False
        self.make_corner_point_pairs()
        self.make_bresenham_lines()
        # if new_point_x <=2 or new_point_x >=245 or new_point_y<=2 or new_point_y >=247:   # boundary check
        #     return False
        return True

    def check_possible_config(self, grid, position, heading):
        temp_robot = Robot(self.width, self.length, self.wheelbase, self.min_turn_radius, self.max_vel, position, heading)
        # print("position in robot", temp_robot.location)
        value = temp_robot.make_robot()
        # print(value)
        if (temp_robot.check_collision(grid) == False and value == True):
            return True
        else:
            return False
    
    def check_collision(self, grid):
        
        for i in self.rasterized_lines:
            for point in i:
                if(grid[int(round(point[0]))][int(round(point[1]))].color == environment.GREEN):
                    return True
        return False

    def draw_robot(self, grid):

        for i in self.rasterized_lines:
            for point in i:
                grid[point[0]][point[1]].make_car()

    
    def clear_robot(self, grid):

        for i in self.rasterized_lines:
            for point in i:
                grid[point[0]][point[1]].clear_car()


    def extinguish_fire(self, grid, patch):
        for burnt in patch.burnt_track:
            if(patch.euclidiean_distance(self.location, burnt) <= 10):
                grid[burnt[0]][burnt[1]].color = environment.GREEN
                patch.burnt_track.remove([burnt[0], burnt[1]])
                patch.extinguish_fire = True
                patch.total_extinguish += 1
        
        # A_star.AStar.q.acla
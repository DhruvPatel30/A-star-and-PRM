import math

wid_rob = 2.2
len_rob = 4.9
wheelbase_rob = 3
min_turn_radius_rob = 13
max_vel_rob = 10 
start_location = (2,2) 
start_orientation = math.pi/6

start = (0,0)
goal = (1,1)

total_sample_points = 400
n_neighbors = 10


global_goal = 0

DISTANCE = []
PREVIOUS = []

G_DISTANCE = []
G_PREVIOUS = []
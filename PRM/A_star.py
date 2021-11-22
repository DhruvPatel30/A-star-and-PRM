import global_variables as g_v
import math
import heapq       
import environment
from PRM import ProbabilisticRoadMap

def calc_dist(start, goal):
    return math.sqrt((goal[0]-start[0])**2 + (goal[1]-start[1])**2)

def AStar( grid, robot, start, goal, patch):
    q = []
    
    g_v.DISTANCE[start[0]][start[1]] = 0
    values = start[0], start[1], robot.orientation
    
    heapq.heappush(q, (g_v.DISTANCE[robot.location[0]][robot.location[1]] + calc_dist(start, goal), values))
    # print("q", q)
    g_v.PREVIOUS[start[0]][start[1]] = start
    delta_t = 0.5
    us_values = [-g_v.max_vel_rob,-g_v.max_vel_rob/2,g_v.max_vel_rob/2, g_v.max_vel_rob]
    max_steer = math.atan2(g_v.wheelbase_rob, g_v.min_turn_radius_rob)
    u_phi_values = [-max_steer,-max_steer/2, 0,max_steer/2,max_steer]
    environment.draw(environment.win, grid, environment.ROWS, environment.width)
    while (len(q) != 0):
        
        priority, u = heapq.heappop(q)
        # u = q[0]
        x = u[0]
        y = u[1]
        # print("local while")
        current_heading = u[2]
        possible_states = []
        # count = 0
        # environment.draw(environment.win, grid, environment.ROWS, environment.width)
        # print("in local")
        for us in (us_values):
            for u_phi in (u_phi_values):
                theta_next = us * (math.tan(u_phi) / g_v.wheelbase_rob) * delta_t + current_heading
                x_next = us * math.cos(theta_next) * delta_t + x
                y_next = us * math.sin(theta_next) * delta_t + y
                position = int(round(x_next)), int(round(y_next))
                heading = theta_next
                # vehicle = []
                # print("position in A_star", position, robot.location)

                if robot.check_possible_config(grid, position, heading):
                    # print("hi")
                    values = position[0], position[1], heading
                    possible_states.append(values)
        
        for v in possible_states:
            if ((g_v.DISTANCE[v[0]][v[1]] > g_v.DISTANCE[x][y] + calc_dist(u, v))):
                # print(v, "v")
                g_v.DISTANCE[v[0]][v[1]] = g_v.DISTANCE[x][y] + calc_dist(u, v)
                heapq.heappush(q, (g_v.DISTANCE[v[0]][v[1]] + (calc_dist(v, goal)), v))
                g_v.PREVIOUS[v[0]][v[1]] = u

        if goal == g_v.global_goal:
            if (calc_dist(u, goal) <= 15):
                new_goal = u
                # print(" Final goal Local A-Star Search complete. Tracing path....")
                trace_path( grid, robot, start, new_goal, patch)
                return (True)
        if (calc_dist(u, goal) <= 15):
            new_goal = u
            # print(" Local A-Star Search complete. Tracing path....")
            trace_path( grid, robot, start, new_goal, patch)
            return True
        # print("q_len", len(q))
        if (len(q) == 0):
            # print("ERROR - Path to the destination not found.")
            return (False)
            
    return True

def trace_path(grid, robot, start, end, patch):
    result = []
    while (end != start):
        # print(start, end)
        result.append(end)
        end = g_v.PREVIOUS[end[0]][end[1]]
        # print("local trace")
        # environment.draw(environment.win, grid, environment.ROWS, environment.width)
    result.append(start)
    result.reverse()
    # print("got_result")
    for ii, i in enumerate(result):
        # position = i[0], i[1]
        robot.location = i[0], i[1]
        # print(robot.location)
        robot.orientation = i[2]
        robot.make_robot()
        robot.draw_robot(grid)
        # grid[i[0]][i[1]].color = environment.PURPLE
        # robot.trace.append([i[0], i[1]])
        environment.draw(environment.win, grid, environment.ROWS, environment.width)
        if(ii != len(result)-1):
            robot.clear_robot(grid)
        
        robot.extinguish_fire(grid, patch)
    
    # print("Done.")
    # return





def global_AStar( grid, robot, start, goal, patch, prm):
    q = []
    
    g_v.G_DISTANCE[start[0]][start[1]] = 0
    values = start[0], start[1]
    
    heapq.heappush(q, (g_v.G_DISTANCE[start[0]][start[1]] + calc_dist(start, goal), values))
    
    g_v.G_PREVIOUS[start[0]][start[1]] = start
    # environment.draw(environment.win, grid, environment.ROWS, environment.width)
    while (len(q) != 0):
        
        priority, u = heapq.heappop(q)
        # x = u[0]
        # y = u[1]
        # print(u, goal)

        possible_states = prm.edges_d[(str(u[0])+','+str(u[1]))]
        # print(possible_states)
        # print("global while")
        # print(len(q))
        for v in possible_states:
            if ((g_v.G_DISTANCE[v[0]][v[1]] > g_v.G_DISTANCE[u[0]][u[1]] + calc_dist(u, v))):
                # print("hii i am here")
                g_v.G_DISTANCE[v[0]][v[1]] = g_v.G_DISTANCE[u[0]][u[1]] + calc_dist(u, v)
                heapq.heappush(q, (g_v.G_DISTANCE[v[0]][v[1]] + (calc_dist(v, goal)), v))
                g_v.G_PREVIOUS[v[0]][v[1]] = u
                # grid[v[0]][v[1]].color = environment.BLACK
        # environment.draw(environment.WIN, grid, environment.ROWS, environment.width)
        if (u == goal):
            new_goal = u
            # print(" Global A-Star Search complete. Tracing path....")
            result = global_trace_path( grid, robot, start, new_goal, patch)
            return result
        # print("q_len", len(q))
        if (len(q) == 0):
            # print("ERROR - Path to the destination not found.")
            return (False)
            
    return True

def global_trace_path(grid, robot, start, end, patch):
    result = []
    while (end != start):
        # print(start, end)
        result.append(end)
        end = g_v.G_PREVIOUS[end[0]][end[1]]
        # print("global trace")
        # environment.draw(environment.win, grid, environment.ROWS, environment.width)
    result.append(start)
    result.reverse()
    # for point in result:
        # grid[point[0]][point[1]].color = environment.BLACK
    # environment.draw(environment.win, grid, environment.ROWS, environment.width)
    # print("Done for global.")
    return result


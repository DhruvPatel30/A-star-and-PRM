
# Wild Fire (Implementation of A* and PRM)

This project involves the implementation of combinatorial (A*) and sampling-based (PRM) motion planning methods, in navigating a firetruck across the obstacle field in attempt to extinguish as many fires as possible.


## Environment

The environment consists of a flat square field, 250 meters on a side, filled with obstacles. The
obstacles consist of large patches of un-navigable thick brush, trees, and weeds.The base dimension for each
obstacle square unit is 15 meters. Inside this field, a firetruck operates, attempting to extinguish
fires that emerge.

Starting at time 0 and at 60 second intervals, an arsonist sets a major conflagration
at a random obstacle. This sets the obstacle state to burning. After 20 seconds in this state, the obstacle sets all obstacles within a 30 meter radius to the state of burning. 

The truck starts at a random point in the map. If the truck stops within 10 meters of a burning
obstacle, it sets the state to extinguished.

Two Planners are used to solve this problem
***
    1. A* Planner
    2. Probabilistic Roadmap (PRM)
***
The figure below shows the environment created. Green patches represent the bushes and blue box represent the car. The dimensions of the car are as follows:
***
    width - 2.2 meters
    length - 4.9 meters
    wheelbase - 3 meters
    minimum turning radius - 13 meters
    maximum velocity - 10 meters per second
***
![](images/world_1.png)

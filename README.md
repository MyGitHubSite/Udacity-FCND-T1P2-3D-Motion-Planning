## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)
---

### Writeup / README
---
### A. Explain the Starter Code
--
#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

### B. Implementing Your Path Planning Algorithm
--
#### 1. Set your global home position

I read in the first line of the csv file, extracted the latitude and longitude, and used them to set the home position with altitude = 0.  

    with open('colliders.csv', 'r') as f:  
        reader = csv.reader(f, delimiter=',')  
        coords = next(reader)  

    lat0 = float(coords[0][5:])               # -122.3974533  
    lon0 = float(coords[1][6:]) =             #   37.7924804  

    self.set_home_position(lon0, lat0, 0)     # longitude, latitude, altitude  

#### 2. Set your current local position

I grabbed the drone's global position and then used that relative to the global home position to calculate the local position.  

    # Get the drone's global position    
    global_position = [self._longitude, self._latitude, self._altitude]  

    # Convert to current local position using global_to_local()  
    curr_local_position = global_to_local(global_position, self.global_home)  

#### 3. Set grid start position from local position

To figure out the start location I read in the data file and calculated the search grid as well as the north and east offsets.  The grid start was set equal to the local poistion minus the offsets.  

    # Read in obstacle map
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

    # Define a grid for a particular altitude and safety margin around obstacles
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    # Convert start position to current position rather than map center
    grid_start = (int(-north_offset + curr_local_position[0]),
                      int(-east_offset + curr_local_position[1]))

#### 4. Set grid goal position from geodetic coords

A geodetic goal was set using coordinates close to the start position.  This goal relative to global home was converted to local coordinates.  The grid goal was set equal to the local coordinates minus the offsets.  

    goal = [-122.3976, 37.7923, 0]  

    local_goal = global_to_local(goal, self.global_home)  

    grid_goal = (int(-north_offset + local_goal[0]),  
                 int(-east_offset + local_goal[1]))  


#### 5. Modify A* to include diagonal motion (or replace A* altogether)

I modified the planning_utils.py file to include diagonal motion that could be used within the A* grid search function.

Within planning_utils.py I added the following block of code to the *Action* class:

    NORTHWEST = (-1, -1, pow(2, 0.5))
    NORTHEAST = (-1, 1, pow(2, 0.5))
    SOUTHWEST = (1, -1, pow(2, 0.5))
    SOUTHEAST = (1, 1, pow(2, 0.5))

Within planning_utils.py I added the following block of code to the *valid_actions* functions:

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if (x - 1 < 0 or y + 1 < 0) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if (x + 1 < 0 or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    if (x + 1 < 0 or y + 1 < 0) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)

#### 6. Cull waypoints 

To cull waypoints which fell in a straight line or very close to straight line I created three functions in the planning_utils.py file.  The points in a path were put into a 3x3 matrix and then the determinant was calculated using the colinearity check.  If the determinant fell below a threshold (epsilon) the path was deemed to be a line and the middle point in the path was disgarded.

    def point(p):  
        return np.array([p[0], p[1], 1.]).reshape(1, -1)  

    def collinearity_check(p1, p2, p3, epsilon=1e-6):  
        m = np.concatenate((p1, p2, p3), 0)  
        det = np.linalg.det(m)                    # calculate the determinant  
        return abs(det) < epsilon  

    def prune_path(path):  
        pruned_path = [p for p in path]  

        i = 0  
        while i < len(pruned_path) - 2:           # check to make sure there are 3 points in a path  
            p1 = point(pruned_path[i])  
            p2 = point(pruned_path[i+1])  
            p3 = point(pruned_path[i+2])  

            if collinearity_check(p1, p2, p3):  
                pruned_path.remove(pruned_path[i+1])  
            else:  
                i += 1  
        return pruned_path  
        
The prune path function was called after a path was found using A* grid search in the motion_planning.py file.  The waypoints were calculated from the pruned path.  

      path, cost = a_star(grid, heuristic, grid_start, grid_goal)  

      # Prune path to minimize number of waypoints  
      pruned_path = prune_path(path)  
      
      # Convert path to waypoints  
      waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in pruned_path]  
---
### C. Execute the flight

#### 1. Does it work?
It works!

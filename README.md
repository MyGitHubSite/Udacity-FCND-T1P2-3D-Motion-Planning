## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)
---

### Writeup / README

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

### Implementing Your Path Planning Algorithm

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
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

I modified the 

Within planning_utils.py I added the following block of code to the *Action* class:

    NORTHWEST = (-1, -1, pow(2, 0.5))
    NORTHEAST = (-1, 1, pow(2, 0.5))
    SOUTHWEST = (1, -1, pow(2, 0.5))
    SOUTHEAST = (1, 1, pow(2, 0.5))

Within planning_utils.py I added the following block of code to the *valid_actions* functions:

def valid_actions

    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTHWEST)
    if (x - 1 < 0 or y + 1 < 0) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTHEAST)
    if (x + 1 < 0 or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTHWEST)
    if (x + 1 < 0 or y + 1 < 0) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTHEAST)



#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

### Execute the flight
#### 1. Does it work?
It works!

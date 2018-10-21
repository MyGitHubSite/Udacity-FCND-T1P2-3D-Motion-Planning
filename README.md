## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---

# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].



## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

I read in the first line of the csv file, extracted the latitude and longitude, and used them to set the home position with altitude = 0.

Code:  

    with open('colliders.csv', 'r') as f:  
        reader = csv.reader(f, delimiter=',')  
        coords = next(reader)  

    lat0 = float(coords[0][5:])               # -122.3974533  
    lon0 = float(coords[1][6:]) =             #   37.7924804  

    self.set_home_position(lon0, lat0, 0)     # longitude, latitude, altitude  


#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

I grabbed the drone's global position and then used that relative to the global home position to calculate the local position.  

Code:  

    # Get the drone's global position    
    global_position = [self._longitude, self._latitude, self._altitude]  

    # Convert to current local position using global_to_local()  
    curr_local_position = global_to_local(global_position, self.global_home)  


#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

To figure out the start location I read in the data file and calculated the search grid as well as the north and east offsets.  The grid start was set equal to the local poistion minus the offsets.  

Code:  

    # Read in obstacle map
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

    # Define a grid for a particular altitude and safety margin around obstacles
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    # Convert start position to current position rather than map center
    grid_start = (int(-north_offset + curr_local_position[0]),
                      int(-east_offset + curr_local_position[1]))


#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

A geodetic goal was set using coordinates close to the start position.  This goal relative to global home was converted to local coordinates.  The grid goal was set equal to the local coordinates minus the offsets.  

Code:  

    goal = [-122.3976, 37.7923, 0]  

    local_goal = global_to_local(goal, self.global_home)  

    grid_goal = (int(-north_offset + local_goal[0]),  
                 int(-east_offset + local_goal[1]))  


#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.




#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

### Execute the flight
#### 1. Does it work?
It works!

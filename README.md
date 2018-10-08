## Project: 3D Motion Planning

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

In `motion_planning.py` drone is set to manual mode (`MANUAL` state) and armed (`ARMING` state).

After drone is armed (`ARMING` state), drone flight path is constructed (state `PLANNING`).

In position change callback, after drone is taken off (`TAKEOFF` state), drone is transitioned through planned waypoints (state `WAYPOINT`). 

When goal endpoint is reached drone is landed (`LANDING`) and disarmed (`DISARMING`).

In `planning_utils.py` grid representation is constructed using given obstacle data. A* search implemented is provided without waypoint pruning. 

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

Longitude and latitue is extracted from *colliders.csv* first line. This can be found in file `motion_planning.py` on lines 127-134: 

```python
with open('colliders.csv') as f:
            first_line = f.readline()
            result = re.findall("[-+]?\d+\.\d+", first_line)
            lat0 = float(result[0])
            lon0 = float(result[1])
         
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)
```

#### 2. Set your current local position

This is accomplished in file `motion_planning.py` on line 138:
```python
current_local_pos = global_to_local(self.global_position, self.global_home)
```

#### 3. Set grid start position from local position

This is accomplished in file `motion_planning.py` on line 150:
```python
grid_start = (int(np.ceil(current_local_pos[0] - north_offset)), int(np.ceil(int(current_local_pos[1]) - east_offset)))
```

#### 4. Set grid goal position from geodetic coords

This is accomplished in file `motion_planning.py` on line 153-154:
```python
goal_north, goal_east, goal_alt = global_to_local(self.target_position_global, self.global_home)
grid_goal = (int(np.ceil(goal_north - north_offset)), int(np.ceil(goal_east - east_offset)))
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

Diagonal motion is done in `planning_utils.py` on line 61-64 by adding diagonal movements with costs *sqrt(2)*:
```python
NORTH_EAST = (-1, 1, sqrt(2))
NORTH_WEST = (-1, -1, sqrt(2))
SOUTH_EAST = (1, 1, sqrt(2))
SOUTH_WEST = (1, -1, sqrt(2))
```

and on lines 94-101:
```python
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTH_EAST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTH_EAST)
if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
    valid_actions.remove(Action.SOUTH_WEST)
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTH_WEST)
```

#### 6. Cull waypoints 
My implementation go over all points found in path done by A* and constracts all points using bresenham method and
checks whether there is some collision. 

If there is there is collision, point is added to list of pruned path points.
If there is there is no collision next point is considered for collisions.

Code for this can be found in `planning_utils.py` method `prune_path` on lines 105-109:

```python
def prune_path(path, grid):

    pruned_path = []
    start_point = path[0]
    pruned_path.append(start_point)

    for end_point in path[1:]:
        points = bresenham(start_point, end_point)

        if any(grid[pl[0], pl[1]] == 1 for pl in points):
            start_point = end_point
            pruned_path.append(end_point)

    end_point = path[-1]
    pruned_path.append(end_point)

    return pruned_path
```

For bresenham method I took implementation from one of the classroom exercises, which supported bresenham only in one
direction and added support for every direction. This can be found in *planning_utils.py* method *bresenham* on lines 123-169:

```python
def bresenham(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    cells = []

    i = x1
    j = y1
    d = 0

    # To support bresenham in every direction, lets set different parameters depending which direction is the line
    if x2 > x1:
        i_diff = 1
        i_pred = lambda i: i < x2
        i_index = lambda i: i
        dx = x2 - x1
    else:
        i_diff = -1
        i_pred = lambda i: i > x2
        i_index = lambda i: i - 1
        dx = x1 - x2

    if y2 > y1:
        j_diff = 1
        j_pred = lambda j: j < y2
        j_index = lambda j: j
        dy = y2 - y1
    else:
        j_diff = -1
        j_pred = lambda j: j > y2
        j_index = lambda j: j - 1
        dy = y1 - y2

    while i_pred(i) and j_pred(j):
        cells.append([i_index(i), j_index(j)])
        if d < dx - dy:
            d += dy
            i += i_diff
        elif d == dx - dy:
            d += dy
            i += i_diff
            d -= dx
            j += j_diff
        else:
            d -= dx
            j += j_diff

    return np.array(cells)
```

### Running motion planning

Drone can be sent to specific global longitude and altitude by giving following command:
```
python motion_planning.py --goal_global_lon -122.396235 --goal_global_lat 37.794941
```

or other example:
```
python motion_planning.py --goal_global_lon -122.401402 --goal_global_lat 37.797222
```

Simulator must be running in `MOTION PLANNING` regime.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from IPython.display import HTML, display
import heapq

def astar_planner(grid, start, goal):
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    
    while open_list:
        _, current = heapq.heappop(open_list)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
            
        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            
            # Check boundaries and ensure target cell is free
            if (0 <= neighbor[0] < grid.shape[0] and 
                0 <= neighbor[1] < grid.shape[1] and 
                grid[neighbor[0], neighbor[1]] == 0):
                
                # Prevent diagonal corner-cutting through tight maze walls
                if dx != 0 and dy != 0:
                    if grid[current[0] + dx, current[1]] == 1 or grid[current[0], current[1] + dy] == 1:
                        continue
                
                cost = 1.414 if dx != 0 and dy != 0 else 1
                tentative_g_score = g_score[current] + cost
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    h_score = np.sqrt((neighbor[0] - goal[0])**2 + (neighbor[1] - goal[1])**2)
                    f_score = tentative_g_score + h_score
                    heapq.heappush(open_list, (f_score, neighbor))
    return None

# --- Full Labyrinth Maze Setup (20x20) ---
grid_size = 20
grid = np.zeros((grid_size, grid_size))

# Main horizontal maze corridors with alternating openings
grid[4, 0:16] = 1   # Opening on the right
grid[8, 4:20] = 1   # Opening on the left
grid[12, 0:16] = 1  # Opening on the right
grid[16, 4:20] = 1  # Opening on the left

# Internal baffle walls forcing the robot to weave inside each corridor
grid[0:2, 7] = 1
grid[2:4, 12] = 1
grid[5:7, 13] = 1
grid[6:8, 7] = 1
grid[9:11, 6] = 1
grid[10:12, 12] = 1
grid[13:15, 13] = 1
grid[14:16, 7] = 1
grid[17:19, 8] = 1
grid[18:20, 14] = 1

start_node = (1, 1)
goal_node = (18, 18)
path = astar_planner(grid, start_node, goal_node)

# --- 3D Maze & Trail Animation Setup ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Extrude 3D maze walls (semi-transparent so the trail is always visible)
obs_y, obs_x = np.where(grid == 1)
wall_height = 2.0
ax.bar3d(obs_x - 0.5, obs_y - 0.5, np.zeros_like(obs_x), 
         1, 1, wall_height, color='slategray', alpha=0.55, edgecolor='darkslategray')

# Plot Start and Final Goal markers
ax.scatter(start_node[1], start_node[0], 0.5, color='limegreen', s=150, label='Start (1,1)')
ax.scatter(goal_node[1], goal_node[0], 0.5, color='red', marker='*', s=220, label='Final Goal (18,18)')

if path:
    path_y, path_x = zip(*path)
    
    # Active 3D Robot marker and dynamic glowing breadcrumb trail
    robot, = ax.plot([], [], [], marker='o', color='blue', markersize=11, label='AMR Robot')
    trail, = ax.plot([], [], [], color='darkorange', marker='.', markersize=6, linewidth=3, label='Active Trail')
    
    ax.set_xlim(0, grid_size)
    ax.set_ylim(0, grid_size)
    ax.set_zlim(0, 5)
    ax.set_xlabel('X Corridor (m)')
    ax.set_ylabel('Y Corridor (m)')
    ax.set_zlabel('Height (m)')
    ax.set_title("3D AMR Labyrinth Navigation & Trail Simulation")
    ax.legend(loc='upper left')

    def animate(i):
        # Move the robot to the current step
        robot.set_data([path_x[i]], [path_y[i]])
        robot.set_3d_properties([0.5])
        
        # Draw the trail dynamically behind the robot as it progresses
        trail.set_data(path_x[:i+1], path_y[:i+1])
        trail.set_3d_properties([0.25] * (i + 1))
        
        # Smoothly pan the 3D camera to follow the maze progression
        ax.view_init(elev=42, azim=-75 + (i * 1.2))
        return robot, trail

    # Render animation (interval=100 makes the maze run brisk and smooth)
    anim = FuncAnimation(fig, animate, frames=len(path), interval=100, blit=False)
    plt.close()
    display(HTML(anim.to_jshtml()))
else:
    print("No valid path found!")


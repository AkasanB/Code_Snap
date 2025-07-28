```python
import asyncio
import platform
import pygame
import random
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Swarm Simulation - APF with Obstacle Navigation")

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128, 100)  # Faint gray for trajectory line

# Parameters
NUM_DRONES = 5
DRONE_RADIUS = 5
OBSTACLE_COUNT = 20
PATH_OBSTACLE_COUNT = 5  # Obstacles near straight-line path
OBSTACLE_RADIUS = 20
GOAL_RADIUS = 10
DRONE_SPEED = 3
FORMATION_SCALE = 20
TRAIL_LENGTH = 50
GOAL_THRESHOLD = 15
SENSOR_RANGE = 30
DRONE_SENSOR_RANGE = 20
ATTRACTIVE_GAIN = 1.0
REPULSIVE_GAIN_OBSTACLE = 1500  # Increased for stronger avoidance
REPULSIVE_GAIN_DRONE = 500
FORMATION_GAIN = 0.5
TRAJECTORY_GAIN = 0.5  # For returning to straight-line path

# Obstacle class
class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

# Drone class
class Drone:
    def __init__(self, x, y, id):
        self.x = x
        self.y = y
        self.id = id
        self.trail = []
        self.formation_offset = (0, 0)  # Set in setup

    def distance_to(self, x, y):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    def update_trail(self):
        self.trail.append((self.x, self.y))
        if len(self.trail) > TRAIL_LENGTH:
            self.trail.pop(0)

# Generate obstacles, some on the straight-line path
def generate_obstacles():
    obstacles = []
    # Straight-line path from start to goal
    path_points = []
    steps = 10
    for t in range(1, steps + 1):
        t = t / steps
        x = start_pos[0] + t * (goal_pos[0] - start_pos[0])
        y = start_pos[1] + t * (goal_pos[1] - start_pos[1])
        path_points.append((x, y))
    
    # Place obstacles near the path
    for _ in range(PATH_OBSTACLE_COUNT):
        if path_points:
            px, py = random.choice(path_points[1:-1])  # Avoid start/goal
            x = px + random.uniform(-20, 20)
            y = py + random.uniform(-20, 20)
            if (math.sqrt((x - start_pos[0])**2 + (y - start_pos[1])**2) > 50 and
                math.sqrt((x - goal_pos[0])**2 + (y - goal_pos[1])**2) > 50):
                obstacles.append(Obstacle(x, y, OBSTACLE_RADIUS))
    
    # Add remaining random obstacles
    for _ in range(OBSTACLE_COUNT - PATH_OBSTACLE_COUNT):
        x = random.randint(50, WIDTH - 50)
        y = random.randint(50, HEIGHT - 50)
        if (math.sqrt((x - start_pos[0])**2 + (y - start_pos[1])**2) > 50 and
            math.sqrt((x - goal_pos[0])**2 + (y - goal_pos[1])**2) > 50):
            obstacles.append(Obstacle(x, y, OBSTACLE_RADIUS))
    
    return obstacles

# Start and goal positions
start_pos = (50, 50)
goal_pos = (750, 550)

# Initialize drones in wedge formation
drones = []
for i in range(NUM_DRONES):
    x = start_pos[0] + (i - NUM_DRONES // 2) * FORMATION_SCALE
    y = start_pos[1] + abs(i - NUM_DRONES // 2) * FORMATION_SCALE
    drone = Drone(x, y, i)
    drone.formation_offset = (x - start_pos[0], y - start_pos[1])
    drones.append(drone)

# Check collision
def is_collision_free(x, y, obstacles, radius=DRONE_RADIUS):
    for obs in obstacles:
        dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
        if dist < obs.radius + radius:
            return False
    return True

# Find closest point on straight-line trajectory
def closest_point_on_trajectory(x, y, start, goal):
    sx, sy = start
    gx, gy = goal
    # Vector from start to goal
    dx, dy = gx - sx, gy - sy
    length_sq = dx**2 + dy**2
    if length_sq == 0:
        return sx, sy
    # Project drone position onto the line
    t = max(0, min(1, ((x - sx) * dx + (y - sy) * dy) / length_sq))
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    return proj_x, proj_y

# Compute APF forces
def compute_apf_forces(drone, drones, obstacles, goal):
    fx, fy = 0, 0
    
    # Attractive force to goal
    dist_to_goal = drone.distance_to(goal[0], goal[1])
    if dist_to_goal > 0:
        fx += ATTRACTIVE_GAIN * (goal[0] - drone.x)
        fy += ATTRACTIVE_GAIN * (goal[1] - drone.y)
    
    # Repulsive force from obstacles (stronger lateral push)
    for obs in obstacles:
        dist = drone.distance_to(obs.x, obs.y)
        if 0 < dist < SENSOR_RANGE + OBSTACLE_RADIUS:
            # Vector from drone to obstacle
            dx = drone.x - obs.x
            dy = drone.y - obs.y
            # Project onto perpendicular direction for lateral avoidance
            path_dx = goal[0] - start_pos[0]
            path_dy = goal[1] - start_pos[1]
            path_length = math.sqrt(path_dx**2 + path_dy**2 + 1e-6)
            if path_length > 0:
                path_dx, path_dy = path_dx / path_length, path_dy / path_length
                # Dot product to find parallel component
                parallel = dx * path_dx + dy * path_dy
                # Perpendicular vector = total - parallel
                perp_dx = dx - parallel * path_dx
                perp_dy = dy - parallel * path_dy
                perp_norm = math.sqrt(perp_dx**2 + perp_dy**2 + 1e-6)
                if perp_norm > 0:
                    perp_dx, perp_dy = perp_dx / perp_norm, perp_dy / perp_norm
                    # Apply stronger repulsive force in perpendicular direction
                    force_magnitude = REPULSIVE_GAIN_OBSTACLE / (dist**2 + 1e-6)
                    fx += force_magnitude * perp_dx
                    fy += force_magnitude * perp_dy
    
    # Repulsive force from other drones
    for other_drone in drones:
        if other_drone != drone:
            dist = drone.distance_to(other_drone.x, other_drone.y)
            if 0 < dist < DRONE_SENSOR_RANGE:
                fx += REPULSIVE_GAIN_DRONE * (drone.x - other_drone.x) / (dist**2 + 1e-6)
                fy += REPULSIVE_GAIN_DRONE * (drone.y - other_drone.y) / (dist**2 + 1e-6)
    
    # Formation force
    swarm_center_x = sum(d.x for d in drones) / NUM_DRONES
    swarm_center_y = sum(d.y for d in drones) / NUM_DRONES
    desired_x = swarm_center_x + drone.formation_offset[0]
    desired_y = swarm_center_y + drone.formation_offset[1]
    fx += FORMATION_GAIN * (desired_x - drone.x)
    fy += FORMATION_GAIN * (desired_y - drone.y)
    
    # Trajectory alignment force
    traj_x, traj_y = closest_point_on_trajectory(drone.x, drone.y, start_pos, goal_pos)
    dist_to_traj = drone.distance_to(traj_x, traj_y)
    if dist_to_traj > 0:
        fx += TRAJECTORY_GAIN * (traj_x - drone.x)
        fy += TRAJECTORY_GAIN * (traj_y - drone.y)
    
    return fx, fy

# Global variables
control_center_log = []
reached_goal = False
obstacles = generate_obstacles()

# Main setup
def setup():
    # Log initial positions
    for drone in drones:
        control_center_log.append({
            'drone_id': drone.id,
            'x': drone.x,
            'y': drone.y,
            'timestamp': 0.0,
            'status': 'Initial'
        })

# Main update loop
def update_loop():
    global reached_goal
    
    if reached_goal:
        # Keep displaying final state
        screen.fill(WHITE)
        
        # Draw ideal trajectory
        surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.line(surface, GRAY, start_pos, goal_pos, 2)
        screen.blit(surface, (0, 0))
        
        # Draw start and goal
        pygame.draw.circle(screen, YELLOW, start_pos, GOAL_RADIUS)
        pygame.draw.circle(screen, RED, goal_pos, GOAL_RADIUS)
        font = pygame.font.SysFont(None, 24)
        screen.blit(font.render("Start", True, BLACK), (start_pos[0] - 20, start_pos[1] - 20))
        screen.blit(font.render("Goal", True, BLACK), (goal_pos[0] - 20, goal_pos[1] - 20))
        
        # Draw obstacles
        for obs in obstacles:
            pygame.draw.circle(screen, GREEN, (int(obs.x), int(obs.y)), obs.radius)
        
        # Draw drones and trails
        swarm_center_x = sum(d.x for d in drones) / NUM_DRONES
        swarm_center_y = sum(d.y for d in drones) / NUM_DRONES
        for drone in drones:
            for i in range(1, len(drone.trail)):
                alpha = int(255 * (i / len(drone.trail)))
                color = (0, 0, 255, alpha)
                surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
                pygame.draw.line(surface, color, drone.trail[i-1], drone.trail[i], 1)
                screen.blit(surface, (0, 0))
            pygame.draw.circle(screen, BLUE, (int(drone.x), int(drone.y)), DRONE_RADIUS)
        
        # Display status
        status = f"Swarm Center: ({int(swarm_center_x)}, {int(swarm_center_y)}) - Goal Reached"
        screen.blit(font.render(status, True, BLACK), (10, 10))
        for i, drone in enumerate(drones):
            pos_text = f"Drone {drone.id}: ({int(drone.x)}, {int(drone.y)})"
            screen.blit(font.render(pos_text, True, BLACK), (10, 30 + i * 20))
        
        pygame.display.flip()
        return
    
    # Normal update loop
    screen.fill(WHITE)
    
    # Draw ideal trajectory
    surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    pygame.draw.line(surface, GRAY, start_pos, goal_pos, 2)
    screen.blit(surface, (0, 0))
    
    # Draw start and goal
    pygame.draw.circle(screen, YELLOW, start_pos, GOAL_RADIUS)
    pygame.draw.circle(screen, RED, goal_pos, GOAL_RADIUS)
    font = pygame.font.SysFont(None, 24)
    screen.blit(font.render("Start", True, BLACK), (start_pos[0] - 20, start_pos[1] - 20))
    screen.blit(font.render("Goal", True, BLACK), (goal_pos[0] - 20, goal_pos[1] - 20))
    
    # Draw obstacles
    for obs in obstacles:
        pygame.draw.circle(screen, GREEN, (int(obs.x), int(obs.y)), obs.radius)
    
    # Update and draw drones
    swarm_center_x = sum(d.x for d in drones) / NUM_DRONES
    swarm_center_y = sum(d.y for d in drones) / NUM_DRONES
    
    for drone in drones:
        # Compute APF forces
        fx, fy = compute_apf_forces(drone, drones, obstacles, goal_pos)
        
        # Normalize velocity
        norm = math.sqrt(fx**2 + fy**2 + 1e-6)
        if norm > 0:
            fx, fy = (fx / norm) * DRONE_SPEED, (fy / norm) * DRONE_SPEED
        
        # Update position if collision-free
        new_x = drone.x + fx
        new_y = drone.y + fy
        if is_collision_free(new_x, new_y, obstacles):
            drone.x = new_x
            drone.y = new_y
        
        # Ensure drones stay within bounds
        drone.x = max(DRONE_RADIUS, min(WIDTH - DRONE_RADIUS, drone.x))
        drone.y = max(DRONE_RADIUS, min(HEIGHT - DRONE_RADIUS, drone.y))
        
        # Update trail
        drone.update_trail()
        
        # Log position during travel
        control_center_log.append({
            'drone_id': drone.id,
            'x': drone.x,
            'y': drone.y,
            'timestamp': pygame.time.get_ticks() / 1000.0,
            'status': 'Traveling'
        })
        
        # Draw trail
        for i in range(1, len(drone.trail)):
            alpha = int(255 * (i / len(drone.trail)))
            color = (0, 0, 255, alpha)
            surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            pygame.draw.line(surface, color, drone.trail[i-1], drone.trail[i], 1)
            screen.blit(surface, (0, 0))
        
        # Draw drone
        pygame.draw.circle(screen, BLUE, (int(drone.x), int(drone.y)), DRONE_RADIUS)
    
    # Check if goal reached
    swarm_dist_to_goal = math.sqrt((swarm_center_x - goal_pos[0])**2 + (swarm_center_y - goal_pos[1])**2)
    if swarm_dist_to_goal < GOAL_THRESHOLD:
        reached_goal = True
        # Log final positions
        for drone in drones:
            control_center_log.append({
                'drone_id': drone.id,
                'x': drone.x,
                'y': drone.y,
                'timestamp': pygame.time.get_ticks() / 1000.0,
                'status': 'Goal Reached'
            })
    
    # Display status
    status = f"Swarm Center: ({int(swarm_center_x)}, {int(swarm_center_y)}) {'- Goal Reached' if reached_goal else ''}"
    screen.blit(font.render(status, True, BLACK), (10, 10))
    for i, drone in enumerate(drones):
        pos_text = f"Drone {drone.id}: ({int(drone.x)}, {int(drone.y)})"
        screen.blit(font.render(pos_text, True, BLACK), (10, 30 + i * 20))
    
    pygame.display.flip()

# Main async function
async def main():
    setup()
    while True:
        update_loop()
        await asyncio.sleep(1.0 / 60)

# Run the simulation
if platform.system() == "Emscripten":
    asyncio.ensure_future(main())
else:
    if __name__ == "__main__":
        asyncio.run(main())
```

### Changes Made
- **Obstacles on Path**:
  - Added `generate_obstacles` to place `PATH_OBSTACLE_COUNT=5` obstacles near the straight-line path from (50, 50) to (750, 550) using linear interpolation (10 points, ±20 pixel offsets).
  - Remaining 15 obstacles are randomly placed, avoiding start/goal (50-pixel buffer).
- **Trajectory Alignment**:
  - Added `closest_point_on_trajectory` to find the nearest point on the straight-line path.
  - Introduced a trajectory alignment force (`TRAJECTORY_GAIN=0.5`) to pull drones back to the ideal path after avoiding obstacles.
- **Enhanced Repulsive Forces**:
  - Increased `REPULSIVE_GAIN_OBSTACLE` to 1500 for stronger avoidance.
  - Modified obstacle repulsive force to act primarily perpendicular to the start-goal path, encouraging lateral movement around obstacles (using vector projection).
- **Visualization**:
  - Added a faint gray line (`GRAY = (128, 128, 128, 100)`) to show the ideal straight-line trajectory from start to goal.
- **Preserved Features**:
  - Semi-static wedge formation (`FORMATION_SCALE=20`, `FORMATION_GAIN=0.5`).
  - Position tracking in `control_center_log` with initial, traveling, and goal-reached logs.
  - Collision checks to prevent obstacle intersections.
  - Stop at goal when swarm center is within `GOAL_THRESHOLD=15`.

### How to Use
- **Run the Code**: Execute in a Pyodide environment or local Python with Pygame (e.g., Python 3.13).
- **Simulation Details**:
  - **Drones**: 5 drones start at (50, 50) in a semi-static wedge (`FORMATION_SCALE=20`).
  - **Goal**: Red circle at (750, 550), labeled “Goal”.
  - **Start**: Yellow circle at (50, 50), labeled “Start”.
  - **Obstacles**: 20 green circular obstacles (radius 20), 5 near the straight-line path.
  - **Behavior**:
    - Drones move at 3 pixels/frame (`DRONE_SPEED=3`) based on APF forces.
    - Obstacles within `SENSOR_RANGE=30` trigger lateral repulsive forces for avoidance.
    - Drones avoid each other within `DRONE_SENSOR
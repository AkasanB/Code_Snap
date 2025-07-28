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
pygame.display.set_caption("Drone Swarm Simulation - RRT with Obstacle Detour")

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128)
PURPLE = (128, 0, 128)  # For detour segments

# Parameters
NUM_DRONES = 5
DRONE_RADIUS = 5
OBSTACLE_COUNT = 20
PATH_OBSTACLE_COUNT = 5  # Obstacles placed near initial path
OBSTACLE_RADIUS = 20
GOAL_RADIUS = 10
DRONE_SPEED = 3
FORMATION_SCALE = 20
RRT_STEP_SIZE = 20
RRT_MAX_ITER = 1000
RRT_LOCAL_ITER = 500  # Increased for better detour planning
RRT_GOAL_BIAS = 0.4
TRAIL_LENGTH = 50
WAYPOINT_THRESHOLD = 15
GOAL_THRESHOLD = 15
SENSOR_RANGE = 30

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
        self.path = []
        self.path_index = 0
        self.trail = []

    def distance_to(self, x, y):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    def update_trail(self):
        self.trail.append((self.x, self.y))
        if len(self.trail) > TRAIL_LENGTH:
            self.trail.pop(0)

# RRT Node
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

# Check collision
def is_collision_free(x, y, obstacles, radius=DRONE_RADIUS):
    for obs in obstacles:
        dist = math.sqrt((x - obs.x)**2 + (y - obs.y)**2)
        if dist < obs.radius + radius:
            return False
    return True

# Check if any drone is near an obstacle
def is_near_obstacle(drones, obstacles):
    for drone in drones:
        for obs in obstacles:
            dist = drone.distance_to(obs.x, obs.y)
            if dist < SENSOR_RANGE + OBSTACLE_RADIUS:
                return True
    return False

# Find nearest node
def nearest_node(nodes, x, y):
    min_dist = float('inf')
    nearest = None
    for node in nodes:
        dist = math.sqrt((node.x - x)**2 + (node.y - y)**2)
        if dist < min_dist:
            min_dist = dist
            nearest = node
    return nearest

# Smooth RRT path
def smooth_path(path, obstacles, iterations=150):
    if not path:
        return path
    smoothed = path[:]
    for _ in range(iterations):
        if len(smoothed) < 3:
            break
        i = random.randint(0, len(smoothed) - 3)
        j = i + 2
        p1, p2 = smoothed[i], smoothed[j]
        steps = int(math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2) / 5)
        collision_free = True
        for t in range(1, steps + 1):
            t = t / steps
            x = p1[0] + t * (p2[0] - p1[0])
            y = p1[1] + t * (p2[1] - p1[1])
            if not is_collision_free(x, y, obstacles):
                collision_free = False
                break
        if collision_free:
            smoothed[i+1:j] = []
    return smoothed

# RRT path planning
def plan_rrt(start, goal, obstacles, max_iter=RRT_MAX_ITER):
    nodes = [Node(start[0], start[1])]
    for _ in range(max_iter):
        if random.random() < RRT_GOAL_BIAS:
            x_rand, y_rand = goal
        else:
            x_rand = random.randint(0, WIDTH)
            y_rand = random.randint(0, HEIGHT)
        
        nearest = nearest_node(nodes, x_rand, y_rand)
        if not nearest:
            continue
        
        theta = math.atan2(y_rand - nearest.y, x_rand - nearest.x)
        x_new = nearest.x + RRT_STEP_SIZE * math.cos(theta)
        y_new = nearest.y + RRT_STEP_SIZE * math.sin(theta)
        
        if x_new < 0 or x_new > WIDTH or y_new < 0 or y_new > HEIGHT:
            continue
        if not is_collision_free(x_new, y_new, obstacles):
            continue
        
        new_node = Node(x_new, y_new)
        new_node.parent = nearest
        nodes.append(new_node)
        
        if math.sqrt((x_new - goal[0])**2 + (y_new - goal[1])**2) < GOAL_RADIUS:
            path = []
            node = new_node
            while node:
                path.append((node.x, node.y))
                node = node.parent
            path = path[::-1]
            return smooth_path(path, obstacles)
    
    return None

# Generate obstacles, some on the path
def generate_obstacles(initial_path):
    obstacles = []
    # Place some obstacles near the initial path
    for _ in range(PATH_OBSTACLE_COUNT):
        if initial_path and len(initial_path) > 2:
            idx = random.randint(1, len(initial_path) - 2)
            px, py = initial_path[idx]
            # Offset slightly to ensure obstacle is on/near path
            x = px + random.uniform(-20, 20)
            y = py + random.uniform(-20, 20)
            # Ensure obstacle is not too close to start or goal
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
    drones.append(Drone(x, y, i))

# Global variables
control_center_log = []
original_path = None
current_path = None
reached_goal = False
detour_active = False
detour_start_index = 0
detour_path = None

# Main setup
def setup():
    global original_path, current_path, obstacles, detour_path
    original_path = plan_rrt(start_pos, goal_pos, [])
    obstacles = generate_obstacles(original_path)
    current_path = original_path
    detour_path = None
    if current_path:
        for drone in drones:
            drone.path = current_path
            drone.path_index = 0
        # Log initial positions
        for drone in drones:
            control_center_log.append({
                'drone_id': drone.id,
                'x': drone.x,
                'y': drone.y,
                'timestamp': 0.0,
                'waypoint_index': 0,
                'status': 'Initial'
            })

# Main update loop
def update_loop():
    global reached_goal, current_path, detour_active, detour_start_index, detour_path
    
    if reached_goal:
        # Keep displaying final state
        screen.fill(WHITE)
        
        # Draw start and goal
        pygame.draw.circle(screen, YELLOW, start_pos, GOAL_RADIUS)
        pygame.draw.circle(screen, RED, goal_pos, GOAL_RADIUS)
        font = pygame.font.SysFont(None, 24)
        screen.blit(font.render("Start", True, BLACK), (start_pos[0] - 20, start_pos[1] - 20))
        screen.blit(font.render("Goal", True, BLACK), (goal_pos[0] - 20, goal_pos[1] - 20))
        
        # Draw obstacles
        for obs in obstacles:
            pygame.draw.circle(screen, GREEN, (int(obs.x), int(obs.y)), obs.radius)
        
        # Draw original and current paths
        if original_path:
            for i in range(1, len(original_path)):
                pygame.draw.line(screen, GRAY, original_path[i-1], original_path[i], 2)
        if current_path and detour_active:
            for i in range(1, len(current_path)):
                pygame.draw.line(screen, PURPLE, current_path[i-1], current_path[i], 3)
        
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
        waypoint_text = f"Waypoint: {drones[0].path_index}/{len(current_path) if current_path else 0}"
        screen.blit(font.render(waypoint_text, True, BLACK), (10, HEIGHT - 30))
        detour_text = f"Detour: {'Active' if detour_active else 'Inactive'}"
        screen.blit(font.render(detour_text, True, BLACK), (10, HEIGHT - 50))
        for i, drone in enumerate(drones):
            pos_text = f"Drone {drone.id}: ({int(drone.x)}, {int(drone.y)})"
            screen.blit(font.render(pos_text, True, BLACK), (10, 30 + i * 20))
        
        pygame.display.flip()
        return
    
    # Normal update loop
    screen.fill(WHITE)
    
    # Draw start and goal
    pygame.draw.circle(screen, YELLOW, start_pos, GOAL_RADIUS)
    pygame.draw.circle(screen, RED, goal_pos, GOAL_RADIUS)
    font = pygame.font.SysFont(None, 24)
    screen.blit(font.render("Start", True, BLACK), (start_pos[0] - 20, start_pos[1] - 20))
    screen.blit(font.render("Goal", True, BLACK), (goal_pos[0] - 20, goal_pos[1] - 20))
    
    # Draw obstacles
    for obs in obstacles:
        pygame.draw.circle(screen, GREEN, (int(obs.x), int(obs.y)), obs.radius)
    
    # Draw original and current paths
    if original_path:
        for i in range(1, len(original_path)):
            pygame.draw.line(screen, GRAY, original_path[i-1], original_path[i], 2)
    if current_path and detour_active:
        for i in range(1, len(current_path)):
            pygame.draw.line(screen, PURPLE, current_path[i-1], current_path[i], 3)
    
    # Update and draw drones
    swarm_center_x = sum(d.x for d in drones) / NUM_DRONES
    swarm_center_y = sum(d.y for d in drones) / NUM_DRONES
    
    # Check for obstacle on path or nearby
    detour_needed = False
    if current_path and drones[0].path_index < len(current_path):
        target_x, target_y = current_path[drones[0].path_index]
        if not is_collision_free(target_x, target_y, obstacles) or is_near_obstacle(drones, obstacles):
            detour_needed = True
    
    if detour_needed and not detour_active:
        # Plan local detour to next collision-free waypoint
        detour_start_index = drones[0].path_index
        next_waypoint_index = detour_start_index
        while next_waypoint_index < len(original_path) and not is_collision_free(original_path[next_waypoint_index][0], original_path[next_waypoint_index][1], obstacles):
            next_waypoint_index += 1
        if next_waypoint_index < len(original_path):
            detour_goal = original_path[next_waypoint_index]
            temp_detour_path = plan_rrt((swarm_center_x, swarm_center_y), detour_goal, obstacles, max_iter=RRT_LOCAL_ITER)
            if temp_detour_path:
                detour_path = temp_detour_path
                current_path = original_path[:detour_start_index] + detour_path + original_path[next_waypoint_index:]
                for drone in drones:
                    drone.path = current_path
                    drone.path_index = detour_start_index
                detour_active = True
                control_center_log.append({
                    'drone_id': -1,
                    'x': swarm_center_x,
                    'y': swarm_center_y,
                    'timestamp': pygame.time.get_ticks() / 1000.0,
                    'waypoint_index': detour_start_index,
                    'status': 'Detour Started'
                })
    
    for drone in drones:
        if current_path and drone.path_index < len(drone.path):
            target_x, target_y = drone.path[drone.path_index]
            dist = drone.distance_to(target_x, target_y)
            
            # Compute path-following velocity
            if dist > 0:
                dx = (target_x - drone.x) / dist
                dy = (target_y - drone.y) / dist
                new_x = drone.x + dx * DRONE_SPEED
                new_y = drone.y + dy * DRONE_SPEED
                
                # Only move if new position is collision-free
                if is_collision_free(new_x, new_y, obstacles):
                    drone.x = new_x
                    drone.y = new_y
            
            # Check if swarm center is close to waypoint
            swarm_dist = math.sqrt((swarm_center_x - target_x)**2 + (swarm_center_y - target_y)**2)
            if swarm_dist < WAYPOINT_THRESHOLD:
                drone.path_index += 1
                if detour_active and detour_path and drone.path_index >= detour_start_index + len(detour_path):
                    detour_active = False
                    detour_path = None
                    control_center_log.append({
                        'drone_id': -1,
                        'x': swarm_center_x,
                        'y': swarm_center_y,
                        'timestamp': pygame.time.get_ticks() / 1000.0,
                        'waypoint_index': drone.path_index,
                        'status': 'Detour Ended'
                    })
            
            # Log position during travel
            control_center_log.append({
                'drone_id': drone.id,
                'x': drone.x,
                'y': drone.y,
                'timestamp': pygame.time.get_ticks() / 1000.0,
                'waypoint_index': drone.path_index,
                'status': 'Traveling'
            })
        
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
                    'waypoint_index': drone.path_index,
                    'status': 'Goal Reached'
                })
        
        # Ensure drones stay within bounds
        drone.x = max(DRONE_RADIUS, min(WIDTH - DRONE_RADIUS, drone.x))
        drone.y = max(DRONE_RADIUS, min(HEIGHT - DRONE_RADIUS, drone.y))
        
        # Update trail
        drone.update_trail()
        
        # Draw trail
        for i in range(1, len(drone.trail)):
            alpha = int(255 * (i / len(drone.trail)))
            color = (0, 0, 255, alpha)
            surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            pygame.draw.line(surface, color, drone.trail[i-1], drone.trail[i], 1)
            screen.blit(surface, (0, 0))
        
        # Draw drone
        pygame.draw.circle(screen, BLUE, (int(drone.x), int(drone.y)), DRONE_RADIUS)
    
    # Display status
    status = f"Swarm Center: ({int(swarm_center_x)}, {int(swarm_center_y)}) {'- Goal Reached' if reached_goal else ''}"
    screen.blit(font.render(status, True, BLACK), (10, 10))
    waypoint_text = f"Waypoint: {drones[0].path_index}/{len(current_path) if current_path else 0}"
    screen.blit(font.render(waypoint_text, True, BLACK), (10, HEIGHT - 30))
    detour_text = f"Detour: {'Active' if detour_active else 'Inactive'}"
    screen.blit(font.render(detour_text, True, BLACK), (10, HEIGHT - 50))
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
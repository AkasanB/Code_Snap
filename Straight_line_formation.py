import asyncio
import platform
import pygame
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Swarm Simulation - Decentralized Straight Path")

# Colors
WHITE = (255, 255, 255)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
GRAY = (128, 128, 128, 100)

# Parameters
NUM_DRONES = 5
DRONE_RADIUS = 5
GOAL_RADIUS = 10
DRONE_SPEED = 3
FORMATION_SCALE = 30
TRAIL_LENGTH = 50
GOAL_THRESHOLD = 15
DRONE_SENSOR_RANGE = 30
MIN_SEPARATION = 15
GOAL_VELOCITY_GAIN = 1.0
FORMATION_GAIN = 0.5
SEPARATION_GAIN = 0.5
PATH_GAIN = 0.5

# Drone class
class Drone:
    def __init__(self, x, y, id):
        self.x = x
        self.y = y
        self.id = id
        self.trail = []
        self.formation_offset = (0, 0)
        self.at_goal = False

    def distance_to(self, x, y):
        return math.sqrt((self.x - x)**2 + (self.y - y)**2)

    def update_trail(self):
        self.trail.append((self.x, self.y))
        if len(self.trail) > TRAIL_LENGTH:
            self.trail.pop(0)

# Start and goal positions
start_pos = (50, 50)
goal_pos = (750, 550)

# Initialize drones in straight-line formation perpendicular to path
drones = []
path_dx = goal_pos[0] - start_pos[0]
path_dy = goal_pos[1] - start_pos[1]
path_length = math.sqrt(path_dx**2 + path_dy**2 + 1e-6)
if path_length > 0:
    normal_dx, normal_dy = -path_dy / path_length, path_dx / path_length
else:
    normal_dx, normal_dy = 0, 1
for i in range(NUM_DRONES):
    offset = (i - (NUM_DRONES - 1) / 2) * FORMATION_SCALE
    x = start_pos[0] + offset * normal_dx
    y = start_pos[1] + offset * normal_dy
    drone = Drone(x, y, i)
    drone.formation_offset = (offset * normal_dx, offset * normal_dy)
    drones.append(drone)

# Find closest point on straight-line path
def closest_point_on_path(x, y, start, goal):
    sx, sy = start
    gx, gy = goal
    dx, dy = gx - sx, gy - sy
    length_sq = dx**2 + dy**2
    if length_sq == 0:
        return sx, sy
    t = max(0, min(1, ((x - sx) * dx + (y - sy) * dy) / length_sq))
    proj_x = sx + t * dx
    proj_y = sy + t * dy
    return proj_x, proj_y

# Compute decentralized velocities
def compute_velocities(drone, drones, goal):
    vx, vy = 0, 0

    # Goal-directed velocity
    dist_to_goal = drone.distance_to(goal[0], goal[1])
    if dist_to_goal > GOAL_THRESHOLD and not drone.at_goal:
        vx += GOAL_VELOCITY_GAIN * (goal[0] - drone.x) / (dist_to_goal + 1e-6)
        vy += GOAL_VELOCITY_GAIN * (goal[1] - drone.y) / (dist_to_goal + 1e-6)

    # Separation velocity
    for other_drone in drones:
        if other_drone != drone:
            dist = drone.distance_to(other_drone.x, other_drone.y)
            if 0 < dist < MIN_SEPARATION:
                vx += SEPARATION_GAIN * (drone.x - other_drone.x) / (dist + 1e-6)
                vy += SEPARATION_GAIN * (drone.y - other_drone.y) / (dist + 1e-6)

    # Formation velocity: maintain straight-line formation
    detected_drones = [d for d in drones if d != drone and drone.distance_to(d.x, d.y) < DRONE_SENSOR_RANGE]
    if detected_drones:
        local_center_x = sum(d.x for d in detected_drones) / max(1, len(detected_drones))
        local_center_y = sum(d.y for d in detected_drones) / max(1, len(detected_drones))
        desired_x = local_center_x + drone.formation_offset[0]
        desired_y = local_center_y + drone.formation_offset[1]
        vx += FORMATION_GAIN * (desired_x - drone.x)
        vy += FORMATION_GAIN * (desired_y - drone.y)

    # Path alignment velocity
    path_x, path_y = closest_point_on_path(drone.x, drone.y, start_pos, goal_pos)
    dist_to_path = drone.distance_to(path_x, path_y)
    if dist_to_path > 0 and not drone.at_goal:
        vx += PATH_GAIN * (path_x - drone.x)
        vy += PATH_GAIN * (path_y - drone.y)

    # Normalize velocity to DRONE_SPEED
    norm = math.sqrt(vx**2 + vy**2 + 1e-6)
    if norm > 0:
        vx, vy = (vx / norm) * DRONE_SPEED, (vy / norm) * DRONE_SPEED

    return vx, vy

# Global variables
control_center_log = []
all_at_goal = False

# Main setup
def setup():
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
    global all_at_goal

    if all_at_goal:
        screen.fill(WHITE)
        surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        pygame.draw.line(surface, GRAY, start_pos, goal_pos, 2)
        screen.blit(surface, (0, 0))
        pygame.draw.circle(screen, YELLOW, start_pos, GOAL_RADIUS)
        pygame.draw.circle(screen, RED, goal_pos, GOAL_RADIUS)
        font = pygame.font.SysFont(None, 24)
        screen.blit(font.render("Start", True, BLACK), (start_pos[0] - 20, start_pos[1] - 20))
        screen.blit(font.render("Goal", True, BLACK), (goal_pos[0] - 20, goal_pos[1] - 20))
        for drone in drones:
            for i in range(1, len(drone.trail)):
                alpha = int(255 * (i / len(drone.trail)))
                color = (0, 0, 255, alpha)
                surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
                pygame.draw.line(surface, color, drone.trail[i-1], drone.trail[i], 1)
                screen.blit(surface, (0, 0))
            pygame.draw.circle(screen, BLUE, (int(drone.x), int(drone.y)), DRONE_RADIUS)
        avg_x = sum(d.x for d in drones) / NUM_DRONES
        avg_y = sum(d.y for d in drones) / NUM_DRONES
        status = f"Avg Position: ({int(avg_x)}, {int(avg_y)}) - All Drones at Goal"
        screen.blit(font.render(status, True, BLACK), (10, 10))
        for i, drone in enumerate(drones):
            pos_text = f"Drone {drone.id}: ({int(drone.x)}, {int(drone.y)})"
            screen.blit(font.render(pos_text, True, BLACK), (10, 30 + i * 20))
        pygame.display.flip()
        return

    screen.fill(WHITE)
    surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    pygame.draw.line(surface, GRAY, start_pos, goal_pos, 2)
    screen.blit(surface, (0, 0))
    pygame.draw.circle(screen, YELLOW, start_pos, GOAL_RADIUS)
    pygame.draw.circle(screen, RED, goal_pos, GOAL_RADIUS)
    font = pygame.font.SysFont(None, 24)
    screen.blit(font.render("Start", True, BLACK), (start_pos[0] - 20, start_pos[1] - 20))
    screen.blit(font.render("Goal", True, BLACK), (goal_pos[0] - 20, goal_pos[1] - 20))

    for drone in drones:
        if not drone.at_goal:
            vx, vy = compute_velocities(drone, drones, goal_pos)
            drone.x += vx
            drone.y += vy
            dist_to_goal = drone.distance_to(goal_pos[0], goal_pos[1])
            if dist_to_goal < GOAL_THRESHOLD:
                drone.at_goal = True
                control_center_log.append({
                    'drone_id': drone.id,
                    'x': drone.x,
                    'y': drone.y,
                    'timestamp': pygame.time.get_ticks() / 1000.0,
                    'status': 'Goal Reached'
                })

        drone.x = max(DRONE_RADIUS, min(WIDTH - DRONE_RADIUS, drone.x))
        drone.y = max(DRONE_RADIUS, min(HEIGHT - DRONE_RADIUS, drone.y))
        drone.update_trail()
        if not drone.at_goal:
            control_center_log.append({
                'drone_id': drone.id,
                'x': drone.x,
                'y': drone.y,
                'timestamp': pygame.time.get_ticks() / 1000.0,
                'status': 'Traveling'
            })
        for i in range(1, len(drone.trail)):
            alpha = int(255 * (i / len(drone.trail)))
            color = (0, 0, 255, alpha)
            surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
            pygame.draw.line(surface, color, drone.trail[i-1], drone.trail[i], 1)
            screen.blit(surface, (0, 0))
        pygame.draw.circle(screen, BLUE, (int(drone.x), int(drone.y)), DRONE_RADIUS)

    if all(drone.at_goal for drone in drones):
        all_at_goal = True

    avg_x = sum(d.x for d in drones) / NUM_DRONES
    avg_y = sum(d.y for d in drones) / NUM_DRONES
    status = f"Avg Position: ({int(avg_x)}, {int(avg_y)}) {'- All Drones at Goal' if all_at_goal else ''}"
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
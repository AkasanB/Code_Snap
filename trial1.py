import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parameters
NUM_DRONES = 5
TIME_STEP = 0.1
TOTAL_TIME = 20
MIN_SEPARATION = 1.5
GOAL_RADIUS = 2.0
GOAL_X = 40
BATTERY_DECAY_RATE = 0.01

# Path function
def path(x):
    return np.array([x, 5 * np.sin(0.2 * x)])

def path_direction(x):
    dx = 1e-2
    return (path(x + dx) - path(x)) / dx

# Initialize drones in horizontal formation
base_x = 0
base_point = path(base_x)
base_dir = path_direction(base_x)
base_dir /= np.linalg.norm(base_dir)
base_normal = np.array([-base_dir[1], base_dir[0]])
offsets = [(i - NUM_DRONES // 2) * 2.0 * base_normal for i in range(NUM_DRONES)]
positions = np.array([base_point + offset for offset in offsets])
velocities = np.zeros((NUM_DRONES, 2))
paths = [[] for _ in range(NUM_DRONES)]
distances = np.zeros(NUM_DRONES)
batteries = np.ones(NUM_DRONES)
collisions = np.zeros(NUM_DRONES, dtype=int)
min_distances = np.full(NUM_DRONES, np.inf)

# Visualization
fig, ax = plt.subplots()
scat = ax.scatter(positions[:, 0], positions[:, 1], c='blue')
trail_lines = [ax.plot([], [], lw=1, alpha=0.6)[0] for _ in range(NUM_DRONES)]
text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=9, va='top')
legend_text = ax.text(0.02, 0.02, '', transform=ax.transAxes, fontsize=9, va='bottom')

threshold_circles = [plt.Circle((0, 0), MIN_SEPARATION, color='red', fill=False, linestyle='--', alpha=0.4) for _ in range(NUM_DRONES)]
for circle in threshold_circles:
    ax.add_patch(circle)

start_pos = path(0)
goal_pos = path(GOAL_X)
ax.plot(start_pos[0], start_pos[1], 'go', label='Start')
ax.plot(goal_pos[0], goal_pos[1], 'ro', label='Goal')
ax.legend(loc='upper right')

ax.set_xlim(0, 45)
ax.set_ylim(-10, 10)
ax.set_title("Centralized Swarm with Full Telemetry")

# Update function
def update(frame):
    global positions, velocities

    # Global path target
    lead_x = np.mean(positions[:, 0]) + 1.0
    path_point = path(lead_x)
    tangent = path_direction(lead_x)
    tangent /= np.linalg.norm(tangent)
    normal = np.array([-tangent[1], tangent[0]])

    # Assign target positions in horizontal formation
    targets = np.array([path_point + (i - NUM_DRONES // 2) * 2.0 * normal for i in range(NUM_DRONES)])

    info_lines = []
    for i in range(NUM_DRONES):
        direction = targets[i] - positions[i]
        direction /= np.linalg.norm(direction) + 1e-6

        # Collision avoidance
        for j in range(NUM_DRONES):
            if i != j:
                dist = np.linalg.norm(positions[i] - positions[j])
                min_distances[i] = min(min_distances[i], dist)
                if dist < MIN_SEPARATION:
                    repulsion = (positions[i] - positions[j]) / (dist + 1e-6)
                    direction += 1.0 * repulsion
                    collisions[i] += 1

        direction /= np.linalg.norm(direction) + 1e-6
        velocities[i] = direction
        positions[i] += velocities[i] * TIME_STEP
        distances[i] += np.linalg.norm(velocities[i]) * TIME_STEP
        batteries[i] -= BATTERY_DECAY_RATE * np.linalg.norm(velocities[i]) * TIME_STEP
        batteries[i] = max(batteries[i], 0)
        paths[i].append(positions[i].copy())

        vel_mag = np.linalg.norm(velocities[i])
        info_lines.append(f"Drone {i}: 🔋 {batteries[i]*100:.1f}% | 🏃 {vel_mag:.2f} | 📍 ({positions[i][0]:.1f}, {positions[i][1]:.1f}) | 📏 MinDist: {min_distances[i]:.2f} | ⚠️ {collisions[i]}")
        threshold_circles[i].center = positions[i]

    # Update visuals
    scat.set_offsets(positions)
    for i, line in enumerate(trail_lines):
        trail = np.array(paths[i])
        if trail.shape[0] > 1:
            line.set_data(trail[:, 0], trail[:, 1])

    text.set_text(f"Frame: {frame}")
    legend_text.set_text("Start: green ●\nGoal: red ●\n" + "\n".join(info_lines))

    # Stop condition
    distances_to_goal = np.linalg.norm(positions - goal_pos, axis=1)
    if np.all(distances_to_goal < GOAL_RADIUS):
        print("✅ All drones reached goal. Stopping simulation.")
        ani.event_source.stop()

# Run animation
ani = FuncAnimation(fig, update, frames=int(TOTAL_TIME / TIME_STEP), interval=50)
plt.show()

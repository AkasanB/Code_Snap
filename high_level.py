import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


# Simulation parameters
NUM_DRONES = 5
DRONE_RADIUS = 0.5  # Not directly used, but for reference
MIN_SEPARATION = 1.5
BATTERY_DECAY_RATE = 0.01
TIME_STEP = 0.1
MAX_STEPS = 1000
GOAL_TOLERANCE = 1.0

# Nonlinear path (e.g., sinusoidal)
def path(t):
    return np.array([t, 5 * np.sin(0.1 * t)])

def path_derivative(t):
    return np.array([1, 0.5 * np.cos(0.1 * t)])

# Initialize drones in horizontal formation
positions = np.array([[0, i * 2.0] for i in range(NUM_DRONES)], dtype=float)
velocities = np.zeros_like(positions)
batteries = np.ones(NUM_DRONES) * 100
distances = np.zeros(NUM_DRONES)
collisions = [0] * NUM_DRONES
paths = [[] for _ in range(NUM_DRONES)]  # Optional: for trailing paths if needed

# Goal is the end of the path
goal_t = 100
goal_pos = path(goal_t)

# Visualization setup
fig, ax = plt.subplots()
# Plot the reference path
t_vals = np.linspace(0, 110, 1000)
path_vals = np.array([path(t) for t in t_vals])
ax.plot(path_vals[:, 0], path_vals[:, 1], 'k--', label='Reference Path')
# Scatter for positions with colors for each drone
colors = plt.cm.viridis(np.linspace(0, 1, NUM_DRONES))
scat = ax.scatter(positions[:, 0], positions[:, 1], c=colors, s=100)
# Quiver for velocities
vel_quivers = ax.quiver(
    positions[:, 0], positions[:, 1],
    velocities[:, 0], velocities[:, 1],
    angles='xy', scale_units='xy', scale=1, color='blue'
)
# Text labels for battery percentages
battery_texts = [ax.text(positions[i, 0] + 1, positions[i, 1] + 0.5, f"{batteries[i]:.1f}%", fontsize=8) for i in range(NUM_DRONES)]
ax.set_xlim(-10, 110)
ax.set_ylim(-20, 20)
ax.set_title("Swarm Formation Control with Battery & Collision Tracking")
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.legend()
ax.grid(True)

def update(frame):
    global positions, velocities, batteries, distances
    new_velocities = np.zeros_like(velocities)
    for i in range(NUM_DRONES):
        if i == 0:  # Leader follows the path
            t_proj = positions[0, 0]
            target = path(t_proj)
            control = path_derivative(t_proj) + 1.0 * (target - positions[0])
        else:  # Followers maintain formation relative to previous drone
            offset = np.array([0, 2.0])
            target = positions[i-1] + offset
            control = velocities[i-1] + 1.0 * (target - positions[i])
        # Collision avoidance
        repulsion = np.zeros(2)
        for j in range(NUM_DRONES):
            if i != j:
                diff = positions[i] - positions[j]
                dist = np.linalg.norm(diff)
                if dist < MIN_SEPARATION and dist > 0:
                    repulsion += diff / dist**2
                    collisions[i] += 1
        control += 1.0 * repulsion
        new_velocities[i] = control
    # Update velocities
    velocities = new_velocities
    # Update positions, distances, and batteries
    positions += velocities * TIME_STEP
    for i in range(NUM_DRONES):
        vel_norm = np.linalg.norm(velocities[i])
        distances[i] += vel_norm * TIME_STEP
        batteries[i] -= BATTERY_DECAY_RATE * vel_norm * TIME_STEP
        batteries[i] = max(batteries[i], 0)
        paths[i].append(positions[i].copy())  # Optional: track path for potential plotting
    # Check if goal reached (based on leader)
    if np.linalg.norm(positions[0] - goal_pos) < GOAL_TOLERANCE or frame >= MAX_STEPS:
        ani.event_source.stop()
        print("\n🧾 Simulation Summary:")
        for i in range(NUM_DRONES):
            print(f"Drone {i+1}:")
            print(f" Final Position: {positions[i]}")
            print(f" Final Velocity: {velocities[i]}")
            print(f" Distance Traveled: {distances[i]:.2f}")
            print(f" Collisions: {collisions[i]}")
            print(f" Battery Remaining: {batteries[i]:.2f}%")
        # Compute the theoretical path distance using integration
        from scipy.integrate import quad
        def path_speed(t):
            dx, dy = 1, 0.5 * np.cos(0.1 * t)
            return np.sqrt(dx**2 + dy**2)
        path_length, _ = quad(path_speed, 0, goal_t)
        print(f" Total Path Distance: {path_length:.2f}")
        return scat, vel_quivers, *battery_texts
    # Update visualization
    scat.set_offsets(positions)
    vel_quivers.set_offsets(positions)
    vel_quivers.set_UVC(velocities[:, 0], velocities[:, 1])
    for i, txt in enumerate(battery_texts):
        txt.set_position((positions[i, 0] + 1, positions[i, 1] + 0.5))
        txt.set_text(f"{batteries[i]:.1f}%")
    return scat, vel_quivers, *battery_texts

ani = FuncAnimation(fig, update, interval=100, blit=True, save_count=MAX_STEPS)
plt.show()

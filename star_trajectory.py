import numpy as np
import matplotlib.pyplot as plt

class Star:
    def __init__(self, mass, position, velocity):
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.force = np.zeros(2, dtype=float)

def calculate_gravitational_force(star1, star2, G=6.67430e-11):
    distance_vector = star2.position - star1.position
    distance = np.linalg.norm(distance_vector)
    if distance == 0:
        return np.zeros(2)
    force_magnitude = G * star1.mass * star2.mass / distance**2
    force_vector = force_magnitude * (distance_vector / distance)
    return force_vector

def update_positions_and_velocities(stars, dt):
    for star in stars:
        star.force = np.zeros(2)
    
    for i, star1 in enumerate(stars):
        for j, star2 in enumerate(stars):
            if i != j:
                star1.force += calculate_gravitational_force(star1, star2)
    
    for star in stars:
        acceleration = star.force / star.mass
        star.velocity += acceleration * dt
        star.position += star.velocity * dt

def simulate(stars, time_steps, dt):
    positions = []
    for _ in range(time_steps):
        update_positions_and_velocities(stars, dt)
        positions.append([star.position.copy() for star in stars])
    return positions

def plot_trajectories(positions):
    for i, trajectory in enumerate(zip(*positions)):
        trajectory = np.array(trajectory)
        plt.plot(trajectory[:, 0], trajectory[:, 1], label=f"Star {i+1}")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.legend()
    plt.title("Star Trajectories")
    plt.show()

# Example usage
if __name__ == "__main__":
    star1 = Star(mass=1e30, position=[0, 0], velocity=[0, 0])
    star2 = Star(mass=1e30, position=[1e11, 0], velocity=[0, 30000])
    stars = [star1, star2]

    time_steps = 1000
    dt = 1000  # seconds

    positions = simulate(stars, time_steps, dt)
    plot_trajectories(positions)
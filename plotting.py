
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.patches import Polygon


def plot_horizontal(x_coordinates, y_coordinates):
    plt.plot(x_coordinates, y_coordinates)

    plt.axhline(y=0, color = 'black', linestyle = '--', label = 'Ground')

    x_values = [-1, 1]
    y_values = [0,0]

    plt.plot(x_values, y_values, color = 'orange', label = 'Landing Pad')

    plt.scatter(x_coordinates[0], y_coordinates[0], color = 'green', label = 'Start')
    plt.scatter(x_coordinates[-1], y_coordinates[-1], color = 'red', label = 'Touchdown')

    plt.xlabel("Horizontal Position (meters)")
    plt.ylabel ("Vertical Position (meters)")
    plt.axis('equal')

    plt.legend()
    plt.show()
    


def plot_vertical(times, altitudes):

    plt.plot(times, altitudes)

    plt.axhline(y=0, color = 'black', linestyle = '--', label = 'Ground')

    plt.scatter(times[0], altitudes[0], color = 'green', label = 'TouchDown')
    plt.scatter(times[-1], altitudes[-1], color = 'red', label = 'Start')

    plt.xlabel("Time (seconds)")
    plt.ylabel ("Altitude (meters)")

    plt.legend()
    plt.show()


def plot_rotation(times, theta_history, omega_history, desired_theta_history):

    # Actual vs desired tilt
    plt.figure()
    plt.plot(times, theta_history, label="Actual Tilt", color="blue")
    plt.plot(times, desired_theta_history, label="Desired Tilt", color="orange", linestyle="--")
    plt.axhline(y=0, color="black", linestyle=":", label="Upright")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Tilt Angle (radians)")
    plt.title("Rocket Tilt: Actual vs Desired")
    plt.legend()
    plt.grid(True)
    plt.show()

    # Angular velocity
    plt.figure()
    plt.plot(times, omega_history, label="Angular Velocity", color="purple")
    plt.axhline(y=0, color="black", linestyle="--")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Angular Velocity (rad/s)")
    plt.title("Rocket Angular Velocity vs Time")
    plt.legend()
    plt.grid(True)
    plt.show()


def plot_fuel_mass(times, fuel_history, mass_history):

    plt.figure()

    plt.plot(times, fuel_history, label="Fuel Remaining (kg)", color="orange")
    plt.plot(times, mass_history, label="Total Mass (kg)", color="blue")

    plt.xlabel("Time (seconds)")
    plt.ylabel("Mass (kg)")
    plt.title("Rocket Fuel and Total Mass vs Time")

    plt.legend()
    plt.grid(True)
    plt.show()



def animate_rocket(trajectory, theta_history):

    fig, ax = plt.subplots()

    # Extract positions from trajectory
    x_positions = [item[0][0] for item in trajectory]
    y_positions = [item[0][1] for item in trajectory]

    ax.set_xlim(min(x_positions) - 2, max(x_positions) + 2)
    ax.set_ylim(min(0, min(y_positions) - 1), max(y_positions) + 3)

    ax.set_aspect('equal')
    ax.set_xlabel("Horizontal Position (m)")
    ax.set_ylabel("Altitude (m)")
    ax.set_title("Rocket Landing Simulation")

    # plot the ground and landing pad
    ax.axhline(y=0, color='black', linestyle='--', label='Ground')
    ax.plot([-1, 1], [0, 0], color='orange',
            linewidth=5, label='Landing Pad')

    # Rocket shape (very basic for the moment)
    rocket_shape = np.array([
        [0, 0.6],
        [-0.15, -0.3],
        [0, -0.2],
        [0.15, -0.3]
    ])

    rocket = Polygon(rocket_shape, closed=True, color='red')
    ax.add_patch(rocket)

    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    ax.legend()

    def update(frame):

        x = x_positions[frame]
        y = y_positions[frame]
        theta = theta_history[frame]

        rotation = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])

        rotated_shape = rocket_shape @ rotation.T

        rotated_shape[:, 0] += x
        rotated_shape[:, 1] += y

        rocket.set_xy(rotated_shape)

        time_text.set_text(f"Time: {(frame + 1) * 0.02:.2f} s")

        return rocket, time_text

    ani = animation.FuncAnimation(
        fig,
        update,
        frames=len(trajectory),
        interval=20,
        blit=True,
        repeat=False
    )

    plt.show()

    return ani

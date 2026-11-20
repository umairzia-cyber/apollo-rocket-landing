
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.patches import Polygon

# ==========================================
# APOLLO - SIMULATION CONFIGURATION
# ==========================================

# TIME
TIMESTEP = 0.02
MAX_SIMULATION_STEPS = 100000

# INITIAL CONDITIONS
INITIAL_POSITION = (0, 10)
INITIAL_VELOCITY = (-1, 0)
INITIAL_THETA = 0.0
INITIAL_OMEGA = 0.0

# MASS AND FUEL
DRY_MASS = 1.0
INITIAL_FUEL = 1.0
INITIAL_MASS = DRY_MASS + INITIAL_FUEL
FUEL_CONSUMPTION_RATE = 0.05

# PHYSICS
GRAVITY = 9.81

# PROPULSION
MAX_THRUST = 30.0

# ROTATIONAL DYNAMICS
MAX_TORQUE = 1.0
MOMENT_OF_INERTIA = 0.5

# CONTROLLER GAINS

# Horizontal controller
K_X_POSITION = 0.25
K_X_VELOCITY = 1.0

# Attitude controller
K_THETA = 4.0
K_OMEGA = 4.0

# Vertical controller
VERTICAL_POSITION_GAIN = 0.2
MAX_DESCENT_SPEED = 0.5
MIN_DESCENT_SPEED = 0.2

# LANDING LIMITS
MAX_HORIZONTAL_ERROR = 1.0
MAX_HORIZONTAL_VELOCITY = 0.5

MIN_VERTICAL_VELOCITY = -1.0
MAX_VERTICAL_VELOCITY = 0.0

MAX_TILT = 0.175
MAX_ROTATION_RATE = 0.175

# VISUALIZATION
LANDING_PAD_HALF_WIDTH = 1.0

# SIMULATION HISTORY
desired_theta_history = []


def pos_ve(position, velocity, theta, omega, fuel_remaining, mass,
           timestep=0.02):

    current_mass = mass

    new_throttle, fuel_remaining, new_mass = vertical_controller(
        position, velocity, current_mass, GRAVITY,
        MAX_THRUST, fuel_remaining, timestep
    )

    theta, omega = rotation_check(
        position, velocity, MAX_TORQUE, MOMENT_OF_INERTIA, theta, omega, timestep
    )

    thrust = new_throttle * MAX_THRUST

    acceleration_x = thrust / current_mass * math.sin(theta)
    acceleration_y = thrust / current_mass * math.cos(theta) - GRAVITY

    velocity_x_new = velocity[0] + acceleration_x * timestep
    velocity_y_new = velocity[1] + acceleration_y * timestep

    position_x_new = position[0] + velocity_x_new * timestep
    position_y_new = position[1] + velocity_y_new * timestep

    final_position = (position_x_new, position_y_new)
    final_velocity = (velocity_x_new, velocity_y_new)

    return (
        final_position,
        final_velocity,
        theta,
        omega,
        fuel_remaining,
        new_mass
    )


def landed_safely(position, velocity, theta, omega):

    if (-MAX_HORIZONTAL_ERROR <= position[0] <= MAX_HORIZONTAL_ERROR) and \
       (-MAX_HORIZONTAL_VELOCITY <= velocity[0] <= MAX_HORIZONTAL_VELOCITY) and \
       (MIN_VERTICAL_VELOCITY <= velocity[1] <= MAX_VERTICAL_VELOCITY) and \
       (-MAX_TILT < theta < MAX_TILT) and \
       (-MAX_ROTATION_RATE < omega < MAX_ROTATION_RATE):

        return True

    return False


def end_simulation(position, velocity, theta, omega):

    if position[1] > 0:
        return False
    else:
        return True

def test(position, velocity, trajectory, trajectory_angle,
         fuel_history, theta, omega, fuel_remaining, mass,
         mass_append, timestep=TIMESTEP):

    initial_fuel = fuel_remaining
    steps = 0

    while True:

        position, velocity, theta, omega, fuel_remaining, mass = pos_ve(
            position, velocity, theta, omega,
            fuel_remaining, mass, timestep
        )

        steps += 1

        # Preserve full simulation history
        trajectory.append((position, velocity))
        trajectory_angle.append((theta, omega))
        fuel_history.append(fuel_remaining)
        mass_append.append(mass)

        if end_simulation(position, velocity, theta, omega):

            if landed_safely(position, velocity, theta, omega):
                status = "Success"
            else:
                status = "Crash"

            break

        if steps >= MAX_SIMULATION_STEPS:
            status = "Step limit"
            break

    # Generate one summary row for this run
    summary = {
        "Status": status,
        "Horizontal Error (m)": abs(position[0]),
        "Horizontal Speed (m/s)": (velocity[0]),
        "Vertical Speed (m/s)": (velocity[1]),
        "Tilt (rad)": (theta),
        "Rotation Rate (rad/s)": (omega),
        "Fuel Burned (kg)": initial_fuel - fuel_remaining,
        "Flight Time (s)": steps * timestep
    }

    return summary


def rotation_check(position, velocity, torque, inertia,
                   theta, omega, timestep=0.02):

    turn_command = attitude_controller(
        position, velocity, theta, omega, torque, inertia
    )

    angular_acceleration = (turn_command * torque) / inertia

    omega_new = omega + angular_acceleration * timestep
    theta_new = theta + omega_new * timestep

    return theta_new, omega_new


def vertical_controller(position, velocity, mass, gravity,
                        max_thrust, fuel_remaining, timestep=TIMESTEP):

    vy_target = -min(
    MAX_DESCENT_SPEED,
    max(VERTICAL_POSITION_GAIN * position[1], MIN_DESCENT_SPEED)
)

    ay_command = vy_target - velocity[1]

    throttle = (mass * (gravity + ay_command)) / max_thrust
    throttle = max(0, min(1, throttle))

    fuel_remaining, new_mass = fuel_check(
        fuel_remaining, throttle, mass, timestep
    )

    if fuel_remaining <= 0:
        return 0, fuel_remaining, new_mass

    return throttle, fuel_remaining, new_mass


def horizontal_controller(position, velocity, gravity):

    desired_acceleration = (
        -(K_X_POSITION * position[0])
        - (K_X_VELOCITY * velocity[0])
    )

    desired_theta = math.atan(desired_acceleration / gravity)

    desired_theta_history.append(desired_theta)

    return desired_theta


def attitude_controller(position, velocity, theta, omega,
                        max_torque, inertia):

    desired_theta = horizontal_controller(position, velocity, GRAVITY)

    desired_angular_acceleration = (
    K_THETA * (desired_theta - theta) - K_OMEGA * omega
)

    turn_command = (
        desired_angular_acceleration * inertia
    ) / max_torque

    return max(-1, min(turn_command, 1))


def fuel_check(fuel_remaining, throttle, current_mass, timestep):

    requested_fuel = FUEL_CONSUMPTION_RATE * throttle * timestep

    actual_fuel_used = min(fuel_remaining, requested_fuel)

    fuel_remaining -= actual_fuel_used
    current_mass = update_mass(current_mass, actual_fuel_used)

    return max(0, fuel_remaining), current_mass


def update_mass(current_mass, fuel_used):

    current_mass -= fuel_used
    return current_mass


def plot_horizontal(x_coordinates, y_coordinates):
    plt.plot(x_coordinates, y_coordinates)

    plt.axhline(y=0, color = 'black', linestyle = '--', label = 'Ground')

    x_values = [-LANDING_PAD_HALF_WIDTH, LANDING_PAD_HALF_WIDTH]
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

    # Fixed ground and landing pad
    ax.axhline(y=0, color='black', linestyle='--', label='Ground')
    ax.plot([-LANDING_PAD_HALF_WIDTH, LANDING_PAD_HALF_WIDTH], [0, 0], color='orange',
            linewidth=5, label='Landing Pad')

    # Rocket shape
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



def main():

    position = INITIAL_POSITION
    velocity = INITIAL_VELOCITY

    trajectory = []
    trajectory_angle = []
    fuel_history = []
    mass_append = []
    theta_history = []
    omega_history = []

    results = []

    summary = test(
        position,
        velocity,
        trajectory,
        trajectory_angle,
        fuel_history,
        INITIAL_THETA,
        INITIAL_OMEGA,
        INITIAL_FUEL,
        INITIAL_MASS,
        mass_append,
        TIMESTEP
    )

    results.append(summary)

    # Display baseline results
    print("\nBASELINE RESULTS")
    print("-" * 65)

    for key, value in results[0].items():
        if isinstance(value, float):
            print(f"{key}: {value:.4f}")
        else:
            print(f"{key}: {value}")

    print("-" * 65)

    print("Total simulation steps:", len(trajectory))

    # Keep trajectories available for future plots
    print("Trajectory points:", len(trajectory))
    print("Fuel history points:", len(fuel_history))


    # list comprehension to seperate x and y co-ordinates
    x_coordinates = [item[0][0] for item in trajectory]
    y_coordinates = [item[0][1] for item in trajectory]

    # list comprehensions for plotting altitude against time
    altitudes = [item[0][1] for item in trajectory]
    times = [step * TIMESTEP for step in range(1, len(trajectory) + 1)]
        
    # list comprehensions for theta and omega
    theta_history = [item[0] for item in trajectory_angle]
    omega_history = [item[1] for item in trajectory_angle]




    plot_horizontal(x_coordinates, y_coordinates)
    plot_vertical(times, altitudes)
    plot_rotation(times, theta_history, omega_history, desired_theta_history)
    plot_fuel_mass(times, fuel_history, mass_append)
    rocket_animation = animate_rocket(trajectory, theta_history)


main()


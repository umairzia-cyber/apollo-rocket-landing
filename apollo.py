from evaluation import test

from plotting import (
    plot_horizontal,
    plot_vertical,
    plot_rotation,
    plot_fuel_mass,
    animate_rocket
)


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

# SIMULATION HISTORY for theta
desired_theta_history = []


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

    summary, trajectory, trajectory_angle, fuel_history, mass_append = test(
    INITIAL_POSITION,
    INITIAL_VELOCITY,
    INITIAL_THETA,
    INITIAL_OMEGA,
    INITIAL_FUEL,
    INITIAL_MASS,
    INITIAL_FUEL,
    TIMESTEP,
    MAX_SIMULATION_STEPS,
    GRAVITY,
    MAX_THRUST,
    MAX_TORQUE,
    MOMENT_OF_INERTIA,
    VERTICAL_POSITION_GAIN,
    MAX_DESCENT_SPEED,
    MIN_DESCENT_SPEED,
    K_X_POSITION,
    K_X_VELOCITY,
    K_THETA,
    K_OMEGA,
    desired_theta_history,
    MAX_HORIZONTAL_ERROR,
    MAX_HORIZONTAL_VELOCITY,
    MIN_VERTICAL_VELOCITY,
    MAX_VERTICAL_VELOCITY,
    MAX_TILT,
    MAX_ROTATION_RATE
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


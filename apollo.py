from unittest import result
import pandas as pd

from numpy import rint

from evaluation import test
from state import RocketState

from plotting import (
    plot_horizontal,
    plot_vertical,
    plot_rotation,
    plot_fuel_mass,
    animate_rocket
)

from config import (
    TIMESTEP,
    MAX_SIMULATION_STEPS,
    DRY_MASS,
    INITIAL_FUEL,
    FUEL_CONSUMPTION_RATE,
    VERTICAL_VELOCITY_GAIN
)


# INITIAL CONDITIONS
INITIAL_POSITION = (0, 10)
INITIAL_VELOCITY = (0, 0)
INITIAL_THETA = 0.0
INITIAL_OMEGA = 0.0

# MASS AND FUEL
INITIAL_MASS = DRY_MASS + INITIAL_FUEL

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

# position in metres, velocity in m/s, theta in radians, 
# omega in rad/s, and fuel and mass in kg.


def main():

    initial_state = RocketState(
    position=INITIAL_POSITION,
    velocity=INITIAL_VELOCITY,
    theta=INITIAL_THETA,
    omega=INITIAL_OMEGA,
    fuel=INITIAL_FUEL,
    dry_mass=DRY_MASS
)

    trajectory = []
    trajectory_angle = []
    fuel_history = []
    mass_append = []
    theta_history = []
    omega_history = []

    results = []

    simulation_result, trajectory, trajectory_angle, fuel_history, mass_history, time_history = test(
    initial_state,
    "Low-Fuel (Crash)",
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
    MAX_ROTATION_RATE,
    DRY_MASS,
    FUEL_CONSUMPTION_RATE
)


    baseline_results = []

    baseline_results.append(simulation_result.to_dict())

    print("\nBASELINE RESULTS")
    print("-" * 65)

    for key, value in baseline_results[0].items():
        if isinstance(value, float):
            print(f"{key}: {value:.4f}")
        else:
            print(f"{key}: {value}")

    print("-" * 65) # formatting line

    print("Total simulation steps:", len(trajectory))
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


    rocket_animation = animate_rocket(trajectory, theta_history)


    baseline_table = pd.DataFrame(baseline_results)

    print("\nBASELINE COMPARISON")
    print(baseline_table.to_string(index=False))


main()


import state
from state import RocketState
from simulation import pos_ve
from results import SimulationResult
import csv
import os

time_history = [0.0]

def landed_safely(
    state,
    max_horizontal_error,
    max_horizontal_velocity,
    min_vertical_velocity,
    max_vertical_velocity,
    max_tilt,
    max_rotation_rate
):
    if (
        -max_horizontal_error <= state.position[0] <= max_horizontal_error
        and -max_horizontal_velocity <= state.velocity[0] <= max_horizontal_velocity
        and min_vertical_velocity <= state.velocity[1] <= max_vertical_velocity
        and -max_tilt <= state.theta <= max_tilt
        and -max_rotation_rate <= state.omega <= max_rotation_rate
    ):
        return True

    return False


def save_result_to_csv(result, filename="baseline_results.csv"):

    file_exists = os.path.isfile(filename)

    with open(filename, mode="a", newline="") as file:

        writer = csv.DictWriter(
            file,
            fieldnames=result.to_dict().keys()
        )

        # Write column headers only when creating a new file
        if not file_exists or os.path.getsize(filename) == 0:
            writer.writeheader()

        writer.writerow(result.to_dict())


def test(
    state,
    scenario_name,
    initial_fuel,
    timestep,
    max_simulation_steps,
    gravity,
    max_thrust,
    max_torque,
    moment_of_inertia,
    vertical_position_gain,
    max_descent_speed,
    min_descent_speed,
    k_x_position,
    k_x_velocity,
    k_theta,
    k_omega,
    desired_theta_history,
    max_horizontal_error,
    max_horizontal_velocity,
    min_vertical_velocity,
    max_vertical_velocity,
    max_tilt,
    max_rotation_rate,
    DRY_MASS,
    FUEL_CONSUMPTION_RATE
):
    trajectory = []
    trajectory_angle = []
    fuel_history = []
    mass_history = []
    time_history = [0.0]

    status = "Failure"
    steps = 0

    initial_position = state.position
    initial_velocity = state.velocity
    initial_theta = state.theta
    initial_omega = state.omega
    initial_fuel = state.fuel

    while state.position[1] > 0 and steps < max_simulation_steps:

        state = pos_ve(
        state,
        gravity,
        max_thrust,
        max_torque,
        moment_of_inertia,
        timestep,
        vertical_position_gain,
        max_descent_speed,
        min_descent_speed,
        k_x_position,
        k_x_velocity,
        k_theta,
        k_omega,
        desired_theta_history,
        DRY_MASS,
        FUEL_CONSUMPTION_RATE
    )

        trajectory.append((state.position, state.velocity))
        trajectory_angle.append((state.theta, state.omega))
        fuel_history.append(state.fuel)
        mass_history.append(state.mass)

        steps += 1
        time_history.append(steps * timestep)

    # evalute landing conditions if the rocket has touched down
    if state.position[1] <= 0:
        if landed_safely(
            state,
            max_horizontal_error,
            max_horizontal_velocity,
            min_vertical_velocity,
            max_vertical_velocity,
            max_tilt,
            max_rotation_rate
        ):
            status = "Success"
        else:
            status = "Crash"
    else:
        status = "Timed-out"

    result = SimulationResult(
    scenario_name=scenario_name,
    initial_position=initial_position,
    initial_velocity=initial_velocity,
    initial_theta=initial_theta,
    initial_omega=initial_omega,
    initial_fuel=initial_fuel,

    status=status,
    horizontal_error=abs(state.position[0]),
    horizontal_velocity=state.velocity[0],
    vertical_velocity=state.velocity[1],
    touchdown_theta=state.theta,
    touchdown_omega=state.omega,
    fuel_burned=initial_fuel - state.fuel,
    flight_time=steps * timestep
)

    save_result_to_csv(result)

    return result, trajectory, trajectory_angle, fuel_history, mass_history, time_history

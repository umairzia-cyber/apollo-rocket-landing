
from simulation import pos_ve



def landed_safely(
    position,
    velocity,
    theta,
    omega,
    max_horizontal_error,
    max_horizontal_velocity,
    min_vertical_velocity,
    max_vertical_velocity,
    max_tilt,
    max_rotation_rate
):
    if (
        -max_horizontal_error <= position[0] <= max_horizontal_error
        and -max_horizontal_velocity <= velocity[0] <= max_horizontal_velocity
        and min_vertical_velocity <= velocity[1] <= max_vertical_velocity
        and -max_tilt < theta < max_tilt
        and -max_rotation_rate < omega < max_rotation_rate
    ):
        return True

    return False



def test(
    position,
    velocity,
    theta,
    omega,
    fuel_remaining,
    mass,
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
    max_rotation_rate
):
    trajectory = []
    trajectory_angle = []
    fuel_history = []
    mass_history = []

    status = "Failure"
    steps = 0

    while position[1] > 0 and steps < max_simulation_steps:

        (
            position,
            velocity,
            theta,
            omega,
            fuel_remaining,
            mass
        ) = pos_ve(
            position,
            velocity,
            theta,
            omega,
            fuel_remaining,
            mass,
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
            desired_theta_history
        )

        trajectory.append((position, velocity))
        trajectory_angle.append((theta, omega))
        fuel_history.append(fuel_remaining)
        mass_history.append(mass)

        steps += 1

    # Ground contact / touchdown evaluation
    if position[1] <= 0:
        if landed_safely(
            position,
            velocity,
            theta,
            omega,
            max_horizontal_error,
            max_horizontal_velocity,
            min_vertical_velocity,
            max_vertical_velocity,
            max_tilt,
            max_rotation_rate
        ):
            status = "Success"
        else:
            status = "Failure"

    summary = {
        "Status": status,
        "Horizontal Error (m)": abs(position[0]),
        "Horizontal Velocity (m/s)": velocity[0],
        "Vertical Velocity (m/s)": velocity[1],
        "Tilt (rad)": theta,
        "Rotation Rate (rad/s)": omega,
        "Fuel Burned (kg)": initial_fuel - fuel_remaining,
        "Flight Time (s)": steps * timestep
    }

    return (
        summary,
        trajectory,
        trajectory_angle,
        fuel_history,
        mass_history
    )

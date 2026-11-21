
import math


def vertical_controller(
    position,
    velocity,
    mass,
    gravity,
    max_thrust,
    fuel_remaining,
    timestep,
    vertical_position_gain,
    max_descent_speed,
    min_descent_speed,
    fuel_check,
    VERTICAL_VELOCITY_GAIN,
    dry_mass,
    FUEL_CONSUMPTION_RATE
):
    vy_target = -min(
        max_descent_speed,
        max(vertical_position_gain * position[1], min_descent_speed)
    )


    velocity_error = vy_target - velocity[1]
    ay_command = VERTICAL_VELOCITY_GAIN * velocity_error

    throttle = (mass * (gravity + ay_command)) / max_thrust
    throttle = max(0, min(1, throttle))

    fuel_remaining, new_mass = fuel_check(
        fuel_remaining, throttle, dry_mass, timestep, FUEL_CONSUMPTION_RATE
    )

    if fuel_remaining <= 0:
        return 0, fuel_remaining, new_mass

    return throttle, fuel_remaining, new_mass


def horizontal_controller(
    position,
    velocity,
    gravity,
    k_x_position,
    k_x_velocity,
    desired_theta_history
):
    desired_acceleration = (
        -(k_x_position * position[0])
        - (k_x_velocity * velocity[0])
    )

    desired_theta = math.atan(desired_acceleration / gravity)

    desired_theta_history.append(desired_theta)

    return desired_theta


def attitude_controller(
    position,
    velocity,
    theta,
    omega,
    max_torque,
    inertia,
    gravity,
    k_x_position,
    k_x_velocity,
    k_theta,
    k_omega,
    desired_theta_history
):
    desired_theta = horizontal_controller(
        position,
        velocity,
        gravity,
        k_x_position,
        k_x_velocity,
        desired_theta_history
    )

    desired_angular_acceleration = (
        k_theta * (desired_theta - theta) - k_omega * omega
    )

    turn_command = (
        desired_angular_acceleration * inertia / max_torque
    )

    return max(-1, min(turn_command, 1))

import math

from controllers import (
    vertical_controller,
    attitude_controller
)


def update_mass(current_mass, fuel_used):
    current_mass -= fuel_used
    return current_mass


def fuel_check(fuel_remaining, throttle, current_mass, timestep):
    requested_fuel = 0.05 * throttle * timestep

    actual_fuel_used = min(fuel_remaining, requested_fuel)

    fuel_remaining -= actual_fuel_used

    current_mass = update_mass(current_mass, actual_fuel_used)

    return max(0, fuel_remaining), current_mass


def rotation_check(
    position, velocity, torque, inertia, theta, omega, timestep,
    gravity, k_x_position, k_x_velocity, k_theta, k_omega,
    desired_theta_history
):
    turn_command = attitude_controller(
        position,
        velocity,
        theta,
        omega,
        torque,
        inertia,
        gravity,
        k_x_position,
        k_x_velocity,
        k_theta,
        k_omega,
        desired_theta_history
    )

    angular_acceleration = (turn_command * torque) / inertia

    omega_new = omega + angular_acceleration * timestep
    theta_new = theta + omega_new * timestep

    return theta_new, omega_new


def pos_ve(
    position, velocity, theta, omega, fuel_remaining, mass,
    gravity, max_thrust, max_torque, inertia, timestep,
    vertical_position_gain, max_descent_speed, min_descent_speed,
    k_x_position, k_x_velocity, k_theta, k_omega,
    desired_theta_history
):
    current_mass = mass

    new_throttle, fuel_remaining, new_mass = vertical_controller(
        position,
        velocity,
        current_mass,
        gravity,
        max_thrust,
        fuel_remaining,
        timestep,
        vertical_position_gain,
        max_descent_speed,
        min_descent_speed,
        fuel_check
    )

    theta, omega = rotation_check(
        position,
        velocity,
        max_torque,
        inertia,
        theta,
        omega,
        timestep,
        gravity,
        k_x_position,
        k_x_velocity,
        k_theta,
        k_omega,
        desired_theta_history
    )

    thrust = new_throttle * max_thrust

    acceleration_x = thrust / current_mass * math.sin(theta)
    acceleration_y = thrust / current_mass * math.cos(theta) - gravity

    velocity_x_new = velocity[0] + acceleration_x * timestep
    velocity_y_new = velocity[1] + acceleration_y * timestep

    position_x_new = position[0] + velocity_x_new * timestep
    position_y_new = position[1] + velocity_y_new * timestep

    new_position = (position_x_new, position_y_new)
    new_velocity = (velocity_x_new, velocity_y_new)

    return (
        new_position,
        new_velocity,
        theta,
        omega,
        fuel_remaining,
        new_mass
    )

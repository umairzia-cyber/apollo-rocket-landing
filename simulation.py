import math
from config import FUEL_CONSUMPTION_RATE, VERTICAL_VELOCITY_GAIN
from state import RocketState

from controllers import (
    vertical_controller,
    attitude_controller
)
import state


def fuel_check(fuel_remaining, throttle, dry_mass, timestep, FUEL_CONSUMPTION_RATE):
    requested_fuel = FUEL_CONSUMPTION_RATE * throttle * timestep

    actual_fuel_used = min(fuel_remaining, requested_fuel)
    new_fuel = max(0.0, fuel_remaining - actual_fuel_used)

    new_mass = dry_mass + new_fuel

    return new_fuel, new_mass


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
    state,
    gravity,
    max_thrust,
    max_torque,
    inertia,
    timestep,
    vertical_position_gain,
    max_descent_speed,
    min_descent_speed,
    k_x_position,
    k_x_velocity,
    k_theta,
    k_omega,
    desired_theta_history,
    dry_mass,
    FUEL_CONSUMPTION_RATE
):
    
    current_mass = state.mass

    new_throttle, fuel_remaining, new_mass = vertical_controller(
        state.position,
        state.velocity,
        current_mass,
        gravity,
        max_thrust,
        state.fuel,
        timestep,
        vertical_position_gain,
        max_descent_speed,
        min_descent_speed,
        fuel_check,
        VERTICAL_VELOCITY_GAIN,
        dry_mass,
        FUEL_CONSUMPTION_RATE
    )

    expected_mass = dry_mass + fuel_remaining


    theta, omega = rotation_check(
        state.position,
        state.velocity,
        max_torque,
        inertia,
        state.theta,
        state.omega,
        timestep,
        gravity,
        k_x_position,
        k_x_velocity,
        k_theta,
        k_omega,
        desired_theta_history
    )

    thrust = new_throttle * max_thrust

    acceleration_x = thrust / new_mass * math.sin(theta)
    acceleration_y = thrust / new_mass * math.cos(theta) - gravity

    velocity_x_new = state.velocity[0] + acceleration_x * timestep
    velocity_y_new = state.velocity[1] + acceleration_y * timestep

    position_x_new = state.position[0] + velocity_x_new * timestep
    position_y_new = state.position[1] + velocity_y_new * timestep

    new_position = (position_x_new, position_y_new)
    new_velocity = (velocity_x_new, velocity_y_new)

    state.position = new_position
    state.velocity = new_velocity
    state.theta = theta
    state.omega = omega
    state.fuel = fuel_remaining

    assert abs(state.mass - new_mass) < 1e-9


    return state

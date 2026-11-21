# APOLLO — Rocket Landing Simulation

## Overview

APOLLO is a 2D rocket landing simulation that models vertical and horizontal movement, rocket rotation, thrust, fuel consumption, and landing control.

The simulation records the rocket's state at every timestep. Matplotlib is used to visualize the flight path, altitude, rotation, fuel consumption, and mass. An animation replays the recorded trajectory without changing the underlying physics.

## 1. Baseline Configuration

All experiments use the same physical parameters and landing limits unless explicitly stated otherwise.

| Parameter | Value |
|---|---:|
| Initial horizontal position | 0 m |
| Initial altitude | 10 m |
| Initial horizontal velocity | 0 m/s |
| Initial vertical velocity | 0 m/s |
| Initial tilt angle (upright baseline) | 0 rad |
| Initial angular velocity | 0 rad/s |
| Initial fuel | 1 kg |
| Initial total mass | 2 kg |
| Dry mass | 1 kg |
| Maximum thrust | 30 N |
| Maximum torque | 1 N·m |
| Moment of inertia | 0.5 kg·m² |
| Gravity | 9.81 m/s² |
| Timestep | 0.02 s |

## 2. Landing Conditions

A successful landing is evaluated using the following limits:

| Metric | Acceptable range |
|---|---:|
| Horizontal position | -1 to +1 m |
| Horizontal velocity | -0.5 to +0.5 m/s |
| Vertical velocity | -1 to 0 m/s |
| Tilt angle | -0.175 to +0.175 rad |
| Angular velocity | -0.175 to +0.175 rad/s |

## 3. Starting Condition Experiments

Five different starting conditions were tested to evaluate how the controller handles initial disturbances.

### Experiment 1: Upright Baseline

The rocket starts upright with no horizontal displacement or velocity.

| Metric | Result |
|---|---:|
| Status | Success |
| Horizontal error | 0.0000 m |
| Horizontal speed | 0.0000 m/s |
| Vertical speed | 0.2005 m/s |
| Final tilt | 0.0000 rad |
| Rotation rate | 0.0000 rad/s |
| Fuel burned | 0.6593 kg |
| Flight time | 24.4800 s |
| Simulation steps | 1224 |

### Experiment 2: Initial Tilt

The rocket starts with an initial tilt of 0.837 radians.

| Metric | Result |
|---|---:|
| Status | Success |
| Horizontal error | 0.0007 m |
| Horizontal speed | 0.0008 m/s |
| Vertical speed | 0.2006 m/s |
| Final tilt | 0.0000 rad |
| Rotation rate | 0.0001 rad/s |
| Fuel burned | 0.6577 kg |
| Flight time | 24.4000 s |
| Simulation steps | 1220 |

### Experiment 3: Initial Horizontal Displacement

The rocket starts 2 metres away from the landing target.

| Metric | Result |
|---|---:|
| Status | Success |
| Horizontal error | 0.0006 m |
| Horizontal speed | 0.0005 m/s |
| Vertical speed | 0.2006 m/s |
| Final tilt | 0.0001 rad |
| Rotation rate | 0.0000 rad/s |
| Fuel burned | 0.6585 kg |
| Flight time | 24.4400 s |
| Simulation steps | 1222 |

### Experiment 4: Initial Horizontal Velocity

The rocket starts with an initial horizontal velocity of 1 m/s and no horizontal displacement.

| Metric | Result |
|---|---:|
| Status | Success |
| Horizontal error | 0.0001 m |
| Horizontal speed | 0.0008 m/s |
| Vertical speed | 0.2006 m/s |
| Final tilt | 0.0001 rad |
| Rotation rate | 0.0000 rad/s |
| Fuel burned | 0.6568 kg |
| Flight time | 24.3600 s |
| Simulation steps | 1218 |

### Experiment 5: Combined Horizontal Disturbance

The rocket starts 2 metres from the target with an initial horizontal velocity of -1 m/s.

| Metric | Result |
|---|---:|
| Status | Success |
| Horizontal error | 0.0006 m |
| Horizontal speed | 0.0002 m/s |
| Vertical speed | 0.2006 m/s |
| Final tilt | 0.0000 rad |
| Rotation rate | 0.0000 rad/s |
| Fuel burned | 0.6581 kg |
| Flight time | 24.4200 s |
| Simulation steps | 1221 |

## 4. Results Comparison

| Starting condition | Status | Horizontal error (m) | Fuel burned (kg) | Flight time (s) |
|---|---|---:|---:|---:|
| Upright baseline | Success | 0.0000 | 0.6593 | 24.48 |
| Initial tilt (0.837 rad) | Success | 0.0007 | 0.6577 | 24.40 |
| Horizontal displacement (2 m) | Success | 0.0006 | 0.6585 | 24.44 |
| Horizontal velocity (1 m/s) | Success | 0.0001 | 0.6568 | 24.36 |
| Combined disturbance | Success | 0.0006 | 0.6581 | 24.42 |

### Observations

- All five tested scenarios resulted in successful landings.
- Horizontal landing errors remained below 0.001 m.
- Final tilt and rotation rate were close to zero in all experiments.
- Fuel consumption remained between 0.6568 kg and 0.6593 kg.
- Flight duration remained between 24.36 and 24.48 seconds.
- The controller successfully corrected the tested initial tilt and horizontal disturbances.

These results demonstrate the controller's performance under the tested initial conditions. They do not establish performance for every possible starting condition.

## 5. Visualizations

The simulation produces the following visualizations:

- **Rocket trajectory:** Horizontal position against altitude.
- **Altitude vs. time:** Shows the descent profile.
- **Rotation vs. time:** Compares actual tilt with desired tilt and displays angular velocity.
- **Fuel and mass vs. time:** Shows fuel consumption and changing rocket mass.
- **Rocket animation:** Replays the recorded position and orientation over time.

### Animation

The animation uses the recorded trajectory and angle history:

```python
rocket_animation = animate_rocket(trajectory, theta_history)
```

It displays the rocket moving and rotating over a fixed ground and landing pad. It does not run the physics again or change the simulation results.

## 6. How to Run

The project can be run using Visual Studio Code.

1. Clone or download the project.
2. Open the project folder in VS Code.
3. Make sure Python is installed and the Python extension is enabled.
4. Open the VS Code terminal.
5. Install the required libraries:

```bash
pip install numpy matplotlib
```

6. Open the main Python simulation file.
7. Run the script using the Run Python File button or the terminal:

```bash
python apollo.py
```

The simulation will execute, record the trajectory, and generate the plots and animation when their respective functions are called.

## 7. Model Assumptions and Limitations

- The simulation is two-dimensional.
- Gravity is constant at 9.81 m/s².
- The rocket is treated as a simplified rigid body.
- Moment of inertia is fixed.
- Fuel consumption changes the rocket's total mass.
- Thrust and torque are limited by their configured maximum values.
- The simulation uses a fixed timestep of 0.02 seconds.
- The model does not represent full real-world rocket aerodynamics, structural behaviour, or complex atmospheric effects.
- The simulation is intended for educational experimentation and controller evaluation, not real-world flight operation.

## 8. Reproducibility

All five experiments were performed using the same physical parameters and landing limits. Only the initial conditions were changed.

Each experiment records the trajectory and fuel history at every simulation step. This allows the plots and animation to be reproduced from the recorded data.

For consistent comparisons, keep the physical parameters, controller configuration, landing limits, and timestep unchanged when testing additional starting conditions.



## Results and Discussion

### Simulation Results

Six test scenarios were evaluated using the same controller gains, timestep (0.02 s), and dry mass (1.0 kg). Five scenarios used the full initial fuel supply of 1.0 kg, while one used a reduced supply of 0.3 kg to test the system under fuel constraints.

| Scenario | Initial condition | Result | Horizontal error (m) | Vertical velocity at landing (m/s) | Flight time (s) |
|---|---|---|---:|---:|---:|
| Upright Baseline | Position (0, 10), velocity (0, 0) | Success | 0.0000 | -0.2005 | 24.48 |
| Position Error | Position (2, 10), velocity (0, 0) | Success | 0.0006 | -0.2006 | 24.44 |
| Velocity Error | Position (0, 10), velocity (-1, 0) | Success | 0.0001 | -0.2006 | 24.36 |
| Opposing Error | Position (2, 10), velocity (-1, 0) | Success | 0.0006 | -0.2006 | 24.42 |
| Recovering Tilt | Position (0, 10), velocity (0, 0), initial tilt 0.0873 rad | Success | 0.0007 | -0.2006 | 24.40 |
| Low-Fuel Test | Position (0, 10), velocity (0, 0), fuel 0.3 kg | Failure | 0.0000 | -6.9742 | 16.74 |

All five full-fuel scenarios achieved successful landings. Despite different initial horizontal positions, velocities, and one initial attitude disturbance, the rocket reached the landing area with very small horizontal errors. The final vertical velocity was approximately -0.20 m/s across all five runs, indicating that the controller maintained a consistent, gentle descent.

The recovering-tilt scenario also completed successfully, suggesting that the attitude controller can correct a small initial angular disturbance during descent. The similar results across the full-fuel cases indicate that the control system can handle the tested initial conditions without changing its controller gains.

The low-fuel scenario produced a different outcome. The rocket exhausted its available 0.3 kg of fuel and reached the ground with a vertical velocity of approximately -6.97 m/s. This exceeded the permitted landing-speed limit, so the landing checker classified the result as a failure. This demonstrates the importance of sufficient fuel reserves for controlled descent.


### Discussion and Limitations

The results demonstrate that the current control system can correct the tested horizontal position and velocity errors, recover from a small initial tilt, and achieve controlled landings when sufficient fuel is available. The low-fuel test also shows that the system cannot guarantee a safe landing when its available thrust duration is insufficient.

However, this is a simplified two-dimensional simulation, not a complete model of a real rocket. It does not represent atmospheric effects such as wind and air resistance, detailed aerodynamic forces, engine response delays, sensor noise, fuel sloshing, structural limitations, or the full complexity of a real landing surface. The simulation also uses simplified thrust and rotational dynamics.

Therefore, these results demonstrate the behaviour of the implemented physics and control model under selected test conditions. They should not be interpreted as evidence that the controller is ready for real-world rocket flight.

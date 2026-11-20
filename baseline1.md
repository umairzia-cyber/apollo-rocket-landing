Rocket Landing Simulation — Baseline Documentation
Baseline 1.0
2D simulation
SI units

1. Overview
The project simulates a rocket attempting a controlled vertical landing using a simplified two-dimensional physics model.

The controller adjusts:
- Engine throttle to control vertical motion.
- Desired rocket tilt to control horizontal motion.
- Rotation to bring actual tilt towards the desired tilt.
The simulation records the rocket's position, velocity, orientation, angular velocity, fuel consumption and mass throughout flight.
2. Physical model and parameters
Parameter	Baseline value	Unit
Horizontal position, x	0	m
Initial altitude, y	10	m
Horizontal velocity, vx	0	m/s
Vertical velocity, vy	0	m/s
Initial tilt, theta	0.0873	rad
Initial angular velocity, omega	0	rad/s
Timestep	0.02	s
Gravity	9.81	m/s²
Dry mass	1	kg
Initial fuel	1	kg
Initial total mass	2	kg
Maximum thrust	30	N
Maximum torque	1	N·m
Rotational inertia	0.5	kg·m²

The initial tilt of 0.0873 radians is approximately 5 degrees.
3. Controller configuration
Vertical controller
The vertical controller calculates a target descent velocity based on altitude:
\[
v_{y,target}=-\min(0.5,\max(0.2y,0.2))
\]

It then calculates the vertical acceleration command:
\[
a_{y,command}=v_{y,target}-v_y
\]

Throttle is calculated using:
\[
Throttle=\operatorname{clip}
\left(
\frac{m(9.81+a_{y,command})}{30},0,1
\right)
\]

This limits throttle to the range from zero to one.
Horizontal controller
The horizontal controller uses horizontal position and velocity to calculate a desired tilt.

The objective is to correct horizontal displacement and reduce horizontal velocity before touchdown.
Attitude controller
The attitude controller attempts to make actual tilt follow desired tilt while damping rotational motion.

Controller gains: The exact numerical gains are not available in the recorded baseline details. These should be copied directly from the controller definitions in your source code rather than estimated.
4. Landing conditions
The simulation considers the landing successful when all the following conditions are satisfied:
Variable	Acceptable range
Horizontal position, x	-1 to +1 m
Horizontal velocity, vx	-0.5 to +0.5 m/s
Vertical velocity, vy	-1 to 0 m/s
Tilt angle, theta	-0.175 to +0.175 rad
Angular velocity, omega	-0.175 to +0.175 rad/s

The simulation terminates when the rocket reaches ground level (y <= 0). It checks the landing conditions at that point. If the conditions are not satisfied, the landing is classified as a crash.

5. Starting condition results.
BASELINE RESULTS (Upright)
-----------------------------------------------------------------
Status: Success
Horizontal Error (m): 0.0000
Horizontal Speed (m/s): 0.0000
Vertical Speed (m/s): 0.2005
Tilt (rad): 0.0000
Rotation Rate (rad/s): 0.0000
Fuel Burned (kg): 0.6593
Flight Time (s): 24.4800
-----------------------------------------------------------------
Total simulation steps: 1224
Trajectory points: 1224
Fuel history points: 1224




Tilted Case (Theta at 0.837 radians)
-----------------------------------------------------------------
Status: Success
Horizontal Error (m): 0.0007
Horizontal Speed (m/s): 0.0008
Vertical Speed (m/s): 0.2006
Tilt (rad): 0.0000
Rotation Rate (rad/s): 0.0001
Fuel Burned (kg): 0.6577
Flight Time (s): 24.4000
-----------------------------------------------------------------
Total simulation steps: 1220
Trajectory points: 1220
Fuel history points: 1220


BASELINE RESULTS (x = 2)
-----------------------------------------------------------------
Status: Success
Horizontal Error (m): 0.0006
Horizontal Speed (m/s): 0.0005
Vertical Speed (m/s): 0.2006
Tilt (rad): 0.0001
Rotation Rate (rad/s): 0.0000
Fuel Burned (kg): 0.6585
Flight Time (s): 24.4400
-----------------------------------------------------------------
Total simulation steps: 1222
Trajectory points: 1222
Fuel history points: 1222



BASELINE RESULTS (xV = 1, x = 0)
-----------------------------------------------------------------
Status: Success
Horizontal Error (m): 0.0001
Horizontal Speed (m/s): 0.0008
Vertical Speed (m/s): 0.2006
Tilt (rad): 0.0001
Rotation Rate (rad/s): 0.0000
Fuel Burned (kg): 0.6568
Flight Time (s): 24.3600
-----------------------------------------------------------------
Total simulation steps: 1218
Trajectory points: 1218
Fuel history points: 1218


BASELINE RESULTS (x = 2, xV = -1)
-----------------------------------------------------------------
Status: Success
Horizontal Error (m): 0.0006
Horizontal Speed (m/s): 0.0002
Vertical Speed (m/s): 0.2006
Tilt (rad): 0.0000
Rotation Rate (rad/s): 0.0000
Fuel Burned (kg): 0.6581
Flight Time (s): 24.4200
-----------------------------------------------------------------
Total simulation steps: 1221
Trajectory points: 1221
Fuel history points: 1221

All scenarios use the same physical parameters and landing limits unless explicitly changed.

Results status: The scenarios were tested, but exact numerical landing outcomes are not available in the recorded results. Do not report them as successful or failed without checking the simulation output.
6. Baseline plots
The following plots are part of the baseline evaluation.
1. Rocket trajectory: Horizontal position against altitude, including ground, landing pad, starting position and touchdown.
2. Altitude vs time: Shows the descent profile and touchdown.
3. Tilt angle vs time: Compares actual theta with desired theta.
4. Angular velocity vs time: Shows rotational behaviour and damping.
5. Fuel remaining vs time: Tracks fuel consumption.
6. Total mass vs time: Tracks the changing mass of the rocket.
The expected mass relationship is:
\[
m_{\text{total}}=m_{\text{dry}}+m_{\text{fuel}}
\]

For this baseline:
\[
m_{\text{total}}=1+m_{\text{fuel}}
\]

The total mass curve should therefore remain exactly 1 kg above the fuel remaining curve.
7. Assumptions and limitations
This baseline is a simplified simulation, not a high-fidelity aerospace model.
Area	Assumption or limitation
Motion	Two-dimensional motion only
Fuel consumption	Simplified fuel burn model
Rotational inertia	Fixed throughout the simulation
Environment	Constant gravity
Wind	Not modelled
Sensor noise	Not modelled
Thrust	Simplified thrust model with a 30 N maximum
Landing surface	Flat ground at y = 0
Vehicle structure	No structural deformation or aerodynamic modelling

Additional simplifications:
- No three-dimensional movement or sideways rotation.
- No atmospheric turbulence or changing environmental conditions.
- No sensor uncertainty or actuator delay.
- No detailed propellant flow or engine dynamics.
These assumptions make the model suitable for studying basic control behaviour, but they limit how closely it represents a real rocket landing.
8. Reproducibility checklist
Before treating this as a frozen baseline, preserve the following:
- [ ] Source code for the simulation.
- [ ] Exact horizontal and attitude controller gains.
- [ ] Exact tested starting conditions.
- [ ] Numerical results for each test.
- [ ] All six baseline plots.
- [ ] Python and library versions.
- [ ] Confirmation that all tests use the same baseline parameters.
Important missing information to fill in from your code:
1. Horizontal controller gains.
2. Attitude controller gains.
3. Exact combined-disturbance starting values.
4. Numerical outcomes for each test.
5. Maximum simulation step count, if one is configured.

9. Baseline conclusion
Baseline Version 1.0 establishes a reference implementation for a simplified two-dimensional rocket landing simulation. It includes vertical throttle control, horizontal position correction, attitude control and fuel consumption.

The baseline provides a reference for evaluating future improvements in controller tuning, stability, fuel efficiency and landing accuracy.

Future versions should change one major component at a time and compare their results against this baseline using the same initial conditions and landing criteria.

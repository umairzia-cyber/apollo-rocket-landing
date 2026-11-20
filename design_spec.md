### SIMULATION PHYSICS AND METRICS for V0 ###
x increases to the right; y increases upward. a +ive x values shows how right of the pad the rocket currently is and y the vertical distance from the pad
The ground is at y = 0; the pad centre is at x = 0, therefore x and y show the distance of the rocket from the ground and pad
theta = 0 means the rocket is upright. A positive angle means it tilts to the right and a negative shows a left tilt
Gravity points downward, so vertical acceleration from gravity is −9.81 m/s².
In Version 0, the controller knows the rocket’s state and the pad’s location directly. No camera or wind yet.
Throttle creates thrust in the direction the rocket points; the turning command changes its rotation; gravity pulls it downward; using the engine consumes fuel.
### FORMULAE ###
Suppose the rocket has a mass of 2 kg and its engine can produce 30 N of thrust at full throttle. What fraction of full throttle should hold it hovering? 

Using (T = m\g), with (g = 9.81\), at 2 kg and upright, 65.4% throttle should hover.


Acceleration:
ax = T/m sin(theta) 
ay = T/m cost(theta) - 9.81

where T is thrust, m is rocket mass, and theta is tilt.

### RESULTS FOR SIMPLE ITERATIONS ###
Simulation results with titl and no tilt pass effectively. 



### Rotation acceleration ####
angular acceleration = turning torque/moment of inertia
omega_new = previous rotation speed + angular acceleration * timestep
theta_new = previous tilt + omega_new * timestep



vy,target = -min(0.5, 0.2y) 

ay,command = vy,target - vy * 1

throttle = m(9.81 + ay,command)/max throttle (and then we limit throttle to in between 0 and 1)

ax,desired = -Kxx -Kvvx (desired acceleration according to the x position and x velocity of the rocket)

theta, desired = atan (ax,desired/gravity) (desired tilt)

turn,desired = Kp,theta(theta,desired - theta) - Kd,theta (Kp controls how strong it tilts, Kd helps stop that rotation. Adjust accordingly.)





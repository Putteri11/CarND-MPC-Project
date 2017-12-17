# Reflections

## The model

The model attempts to follow a polynomial with coefficients `coeffs` by calculating optimal actuator values (throttle and steer values). That is, it tries to minimze a certain cost function, which usually includes the squared values of the cross track error (CTE), orentation error, and velocity error (against a reference velocity), as well as the actuator values, and the difference between the "current" actuator values and the previous ones. All of these components can be modified by adding some weight into them. My implementation has a weight of 3 in the CTE and orientation error components, a weight of 2000 in the steer value difference component, and a weight of 10 in the throttle value difference component. 

The state of the car consists of the x and y coordinates, the heading direction, and the velocity of the car, as well as the CTE and orientation error of the car. From the initial state the optimizer calculates the next state `N - 1` times using the update equations, or constraints. The constraints are somewhat approximative, because they assume a constant velocity and heading direction (exept when setting the constraint for the same state value).

The total cost which is to be minimized is summed from all the costs of the states. When finally the lower and upper bound values for the constraints and the actuators are set, the optimizer is able to find the optimal actuoator values for each of the states.

## Timestep Length and Elapsed Duration (`N` & `dt`)

For the hyperparameters `N` and `dt`, I chose the values `N = 15` and `dt = 0.05`. I tried some other values for these, but noticed that the car performed best with the same values as in the lesson, `N = 25` and `dt = 0.05`. At this stage I hadn't tuned the cost multipliers, but while I was tuning them I noticed that the vehicle tended to "wobble" a little bit too much, so I tried tuning the parameter `N` down a notch which seemed to reduce the "wobbling" at least a little bit.

## Polynomial Fitting and MPC Preprocessing

Before I fed the vehicle state and the polynomial coefficients into the MPC procedure, I took latency into account (see below), transformed the waypoints as well as the point where the vehicle is to vehicle coordinates. I fit the polynomial with those transformed waypoints, and used it's coefficients to calculate the initial state of the vehicle in vehicle coordinates.

## Model Predictive Control with Latency

I took latency into account with simple approximative update equations for the coordinates of the car `px` and `py`, the heading direction of the car `psi`, and the velocity of the car `v`:

```
px += cos(psi) * v * latency;
py += sin(psi) * v * latency;
psi -= v * delta * deg2rad(25.0) / Lf * latency;
v += acceleration * latency;
```

Here the variable `latency` is set to 0.1 (ms).

These equations improved the vehicles performance on the road significantly. Also, I noticed that multiplying `delta` with `deg2rad(25.0)` reduced the "wobbly" behaviour of the vehicle. 


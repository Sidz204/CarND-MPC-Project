# Model Predictive Control 
In this project, I have implemented Model Predictive Control to drive the car around the track. Basically, it involves writing C++ program that can drive a car around a virtual track in simulated environment using specific waypoints from the track itself.

To complete the project, following steps have been followed:
- Fit a 3rd degree polynomial based on the waypoints and predict the future state based on it.
- Fill in the MPC solver for the optimisation library.
- Implement the MPC Model in MPC.cpp which includes setting variables and constraints.
- Calculate the actuator values i.e steering and throttle based on the current state.
- Set the timestep length and duration
- Tune the parameters and test untill we get good results.

## Reflection
Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

### 1. Model, state, actuators and update equations

- State Vector and model update equations : This represents the movement of the vehicle. For this we need to consider the vehicle x & y coordinates, orientation(ψ)(angle made by the vehicle w.r.t x axis) and the velocity(v) of the vehicle. From this we can predict where the vehicle is after certain time using the vehicle dynamic model equations which are explained below. Since, we are considering this in vehicle coordinate system , the initial x,y & orientation is taken 0. Velocity, previous steering and throttle is taken from simulator
In this the Ipopt optimiser library used, the state space also contains the system error. Two model errors are tracked, the Cross Track Error (cte) or the difference between the path trajectory and the vehicle's current position and also the heading angle error (eψ) which is the difference between the vehicles heading angle and the trajectory paths heading angle.

Equations used for predicting future state and actuators:

 X(t+1) = X(t) + V(t) ∗ cos(ψ(t)) ∗ dt
 Y(t+1) = Y(t) + V(t) ∗ sin(ψ(t)) ∗ dt
 ψ(t+1) = ψ(t) + Lf / V(t) ∗ δ ∗ dt
 V(t+1) = V(t) + a(t) * dt
 cte(t+1) = f(x) - y(t) + V(t) ∗ sin(ψ(t)) ∗ dt
 eψ(t+1) = eψ(t) - Y(t) + V(t) ∗ sin(eψ(t)) ∗ dt
 
 where f(x) is the reference line
 
 operator() method is fg_eval class is responsible for calculating these equations. For the case of simulator, steering angle(δ) is opposite hence we multiply by -1.
 
 
 
 
 

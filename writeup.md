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
 


- Actuators :
We are considering two actuators i.e. steering angle(δ) and throttle(for breaking and accelerate)
steering angles is between [-25,25] and throttle [-1,1] where -1 signifies breaking. These values are sent to the simulator to make changes according to the state vector.


- MPC Cost :
The MPC considers the cte_penalty,epsi_penalty ,car_speed_penalty ,steer_use_penalty,a_use_penalty, steer_change_penalty a_change_penalty. I have set more value in steer_change_penalty since it takes into account future steer angle as well. Also the values in cte_penalty, epsi_penalty is more since we want the vehicle to turn back to its reference trajectory. All these values are found out by try and error. The code from line 50-70 in MPC.cpp shows this cost.
 
 
### 2. Selecting N (time step length) and dt (elapsed duration between time steps) values

From the quiz I found out that if N is too small, the MPC will not be able to predict far enough into the future and will not predict an accurate current state and controls to prevent system overshoot or instabilities. However, a large N predicts unnecessary data for the future. Since MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations, larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory which is called "discretization error".

Initially I used N = 25 and dt=0.05 since it gave good results in MPC quiz but that didn't gave good results making car go out of track initially. Later I thought since the simulator has latency of 100ms  it would be great if I change dt to 0.1 but also it didn't gave good results at first. Tuning the cost penalties got some good result but the trajectory was very long & wiggly. So, tried reducing N to 15 and 10 and finally settled with N=10 and dt=0.1



### 3. Path trajectory Polynomial Fitting and the system MPC Preprocessing.

In the simulator, a path trajectory is supplied to the system in world coordinates, but as the vehicle controls and sensors are relative to the vehicle , the trajectory path(waypoints) must be transformed to vehicle coordinates. This involves rotation and translation between the two coordinate systems which is explained in particle filter project and is accomplished by the following equations.

X_c = trans_x * cos(psi) + trans_y * sin(psi);

Y_c = trans_y * cos(psi) - trans_x * sin(psi);

where is trans_x and trans_y are translated points i.e. (x0 - xp) & (y0 - yp) denoting (x0,y0) as position of observation(waypoint) and (xp,yp) as position of the car both in world/map coordinates.

Once the path trajectory points have been converted into vehicle centric coordinates, a line of best fit is calculated in the form of a quadratic equation. This equation is feed into the MPC for time stepped path prediction. 



### 4. Model Predictive Controller handling a 100-millisecond latency.

In order to account for the system latency, the actuators values, are taken one-time step in advance or at time step t1 from the current state. This means that as the MPC predicts the system from the current state, we act as if the state was in the past when the calculation was first performed. This prevents the model from lagging the system as we already one step ahead.


### Results :
See video of the results from my implementation [with_speed_60](). Just for the sake of checking the limit of the model , I have also tested it with speed 100 [with_speed_100]().The cars goes little wiggly in speed 100 and completes 1 or 2 laps successfully after which it goes off the track.More tuning can certainly help in achieving that speed. 
P.S. :While running my code for checking speed 100 please change ref_v and v_ref values to 100 in main.cpp & MPC.cpp files.



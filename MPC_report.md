# Model Preditive Control for self-driving

#### 1. Project Overview

In this project you'll revisit the lake race track from the Behavioral Cloning and PID controller Projects. This time, however, you'll implement a model preditive controller in C++ to maneuver the vehicle around the track!

This solution uses the IPOPT and CPPAD libraries to calculate an optimal trajectory, and also sends back the associated actuation commands to simulator in order to minimize error between predicted trajectory and the targeted trajectory given by the simulator. The predicted trajectory consists of a short duration's worth of waypoints. They are derived from the car's kinematics model and optimized by reducing the totoal cost associated with model's actual states, controls, reference states etc.

[//]: # (Image References)
[image1]: ./modelEquations.png "Viechle Model Equations"

#### 2. Rubic Points
The questions in project rubic points are addressed in the following sections.
##### 2.1. Kinematics Model
Cars' kinematics model includes six states (four basic states: x position, y position, velocity v, heading angle (psi); two augumented error states: Cross track error - Cte and psi error - epsi). Together with actuator vector inputs (steering angle - δ and throttle - a), the model can be used to predict the states in the next timestep (current time + dt) from current states.

![Model Equations][image1]

##### 2.2. Timestep length and elasped duration
Timestep length (dt) and elapsed duration(N) determines how long the prediction time is and how intensive the computation is. The final values chosen are  0.1(dt) and 10 (N) respectively. This means the MPC controller evaluates and optimizes states and acutator inputs for next 1s' (N*dt = 10*0.1) worth of time to minize the defined costs. It makes a good trade off between computation speed and prediction accuarcy. Tried dt less than 0.1 (0.05/0.02/0.01), resulting unstable car moving trajectory with current MPC tuning/implemenation. Also tried dt greater than 0.1 (0.2/0.5), the results are not as good as 0.1 due to lower sampling/correction rates.

##### 2.3. Polynomial Fitting and MPC Preprocessing
The waypoints are preprocessed by transforming them to the vehicle's coordination system (see main.cpp lines 97-109). This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the heading angle is also zero.

##### 2.4. Model Predictive Control with Latency
Actuation latency is handled in lines 125-133 of main.cpp. States at actuation time were updtaed based on current states and latency (defined as control_delay). Then the updated states were passed to the Solver which computes the optimal trajectory. 
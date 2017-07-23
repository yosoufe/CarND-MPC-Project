# MPC Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)
[image1]: ./Images/Model.png "Model"
[image2]: ./Images/proof.jpg "Proof"


## The Model:
This Project basically is using the bicycle kinematic model of the car. It is considering only accelerations, velocities and positions (also in rotaional manner). The following shows the equations. `x` and `y` are reperesenting the position, `psi` is the heading angle with respect to global `x` axis. `dt` is the time difference between samples and `a_t` is acceleration along the vehicle.

![alt text][image1]

These equations are very trivial to deduce, execpt the third one. The following is a proof to the 3rd equation. Here I obtained the equation for the rate of change of the heading angle with respect to the steering angle.

![alt text][image2]

## Timestep Length and Elapsed Duration (N & dt)
These parameters are important and it is always a comprimise which should be considered to choose them.

### dt:
If it is too high: It will lead to unstability because it is changing the commands too late and the optimisation would be a very difficult task.

If it is too low: The time that the optimiser is looking into the future would be very small then the optimiser would have more difficult task to reach a desired state in less time. This time is called control horizon.

### N:
If it is too high: The number of parameters to be calculated by the optimiser would be too much and it would take a lot of time for each sample to be calculated and it may lose the realtime requirement.

If it is too low: Almost the same would happen as when the `dt` is too low.

## Polynomial Fitting and MPC Preprocessing:

First of all all the waypoints are transfered to local car coordinates. That would cause a lot of ease in calculation. Now the car would be at position `x = 0, y = 0 & psi = 0`. and the cross tracking error `cte` would be only error in `y` direction.

I fitted a 3rd order polynomial to the way points and because of the transformation, the derivative would be only the 2nd coefficient of the polynomial and other terms would be zero because `x = 0, y = 0`.

Moreover becasue of the transformation there is no need of angle normalisation when we are subtracting the desired angle and the current angle because they are small enough always if the car is following the road nicely.

## Model Predictive Control with Latency
I decided to choose the objective function in a way that the controller should be still robust against latency without any further predictions. Because in real world this latency is not always the same and difficult to measure. Otherwise prediction of the car based on the model presented above is very easy and then the predicted states could be passed to the optimiser.

# SP'26 ISR: State-Space Methods for Inverted Pendulum Control

The goal of this ISR was to explore topics related modern control theory and then apply it to an inverted pendulum system. The inverted pendulum is a common and well-studied control system that revolves around controlling an underactuated pendulum-cart system about its unstable equilibrium. This particular system was chosen due to it being one of the most intuitive, easily describable and realizable nonlinear unstable systems, as well as convenience due to having leftover hardware from the ESA Rocky project.

## Repository Information

This repository contains relevant files created and used during this ISR. It is split up into 3 main folders:
  - `matlab`: Contains all relevant MATLAB scripts used for modeling, simulation, controller design, and analysis.
  - `balance_code`: Contains all relevant software needed to balance Rocky. The code is written in Arduino IDE for hardware compatibility, where it can be uploaded with ease.
  - `motor_calibration`: Contains all relevant software needed for system identification for Rocky's motors. The code is written in Arduino IDE for hardware compatibility, where it can be uploaded with ease.

Within the `matlab` folder are 3 sub-folders:
  - `Calibration`: Contains relevant scripts for system identification for both the motors and pendulum.
  - `ControllerDesign`: Contains the nonlinear and nonlinear models, scripts for controller design, as well as scripts to simulate and animate the system.
  - `DataAnalysis`: Contains scripts to analyze experimental data and compare it to simulation data.

## Use Instructions:

To run your own simulations, refer to the `ControllerDesign` sub-folder. 

Open the script `controller_design_1dof.m`, enter system parameters that result from system identification, then follow through the controller design process to find the feedback gain matrix $K$.

Open the script `run_sim_1dof.m`, where there are lines to commment in/out depending on the model type. The initial conditions and integration period may also be adjusted. Once ready, type `run_sim_1dof(K)` in the console to run the simulation of the closed-loop nonlinear model, where results are plotted and an animation is played.

An example animation is shown below:

![step_response](/media/step_response.gif)

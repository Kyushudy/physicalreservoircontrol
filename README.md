# Physical Reservoir Control Program

# MATLAB Version
This code was developed using **MATLAB R2024a**. Versions later than R2024a should also be compatible.

# Required MATLAB Toolboxes
Please ensure that **ALL toolboxes related to Simulink and Simscape** are installed. For convenience and to avoid compatibility issues, we recommend **installing all available toolboxes** during MATLAB installation.

*(Estimated installation time: ~1 hour)*

# Required MATLAB Apps
After installing MATLAB, open the application and navigate to "**APPS**" → "**Get More Apps**". Install the following three packages:
- **Robotics Playground**
- **Robotics Playground Expansion Pack**

*(Estimated installation time: ~10 minutes)*

# Main Simulation Script
To run the simulation and collect data for the pendulum's free fall and swing-up tests using the (simulated) physical reservoir, execute the following script: "**simulation code\PR_test_freedrop_continuous.m**"

# Physical Experiment Setup
Computer 1 (Physical Reservoir + OptiTrack system) runs the following scripts in parallel: "**experiment code\run_receive.m**" and "**experiment code\run_send.m**"

Required installations:
- **MATLAB Support Package for Arduino Hardware**
- **OptiTrack software**

Computer 2 (Underactuated Pendulum Simulation) runs: "**experiment code\run_sim.m**"

Place all files under the `experiment code` folder on **both computers**.

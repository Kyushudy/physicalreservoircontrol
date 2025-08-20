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

# Demo: Main Simulation Script
To run the simulation and collect data for the pendulum's free fall and swing-up tests using the (simulated) physical reservoir, execute the following script: "**simulation code\PR_test_freedrop_continuous.m**"

*(Estimated run time: ~1 minute)*

*(Expected output: see **vid1_training_and_control.mp4**)*

# Reproduction Instructions

To reproduce the data analyzed in **Figure 5** and **Figure 7**, run the following scripts: "**simulation code\PR_data_stability_kick.m**", "**simulation code\PR_data_stability_losetrack.m**", and "**simulation code\PR_data_stability_noise.m**"

Each script will generate output using the pretrained model in "**simulation code\success_l1_8_l2_10.mat**" and save the results into the `data` folder. The repository already includes this `data` folder with the generated datasets.

*(Estimated run time: ~7 days)*

# Mathematical Model

All parameter values mentioned in the manuscript can be found in: "**mathematical model\pd_parameters_orig.m**"

To generate the pole-zero map data demonstrated in **Figure 6**, run: "**mathematical model\pd_model.m**"

# Physical Experiment Setup
Computer 1 (Physical Reservoir + OptiTrack system) runs the following scripts in parallel: "**experiment code\run_receive.m**" and "**experiment code\run_send.m**"

Required installations:
- **MATLAB Support Package for Arduino Hardware**
- **OptiTrack software**

Computer 2 (Underactuated Pendulum Simulation) runs: "**experiment code\run_sim.m**"

Place all files under the `experiment code` folder on **both computers**. For detailed instructions on setting up the physical experiments, please refer to **doc1_experiment_setup.docx**.

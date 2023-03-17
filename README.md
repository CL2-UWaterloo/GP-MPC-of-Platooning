## Mixed-Vehicle Platooning Control with a Gaussian Process Learning-Based Model

This repository contains the code for two papers focused on enhancing safety in mixed-traffic platooning with autonomous and human-driven vehicles. Specifically, the papers are:

* [Improving Safety in Mixed Traffic: A Learning-based Model Predictive Control for Autonomous and Human-Driven Vehicle Platooning](https://arxiv.org/abs/2211.04665) 
* [Learning-Based Modeling of Human-Autonomous Vehicle Interaction for Enhancing Safety in Mixed-Vehicle Platooning Control](https://arxiv.org/abs/2303.09452) 

### ARX_simulation
The ARX_simulation folder contains the code for simulating a platoon of vehicles using MPC with an ARX model, where the simulated vehicle is an ARX+GP model. Specifically, the following files are included:
* `MPC_Platoon_simulation`: the main file for running the simulation
* `MPC_Platoon`: the MPC
* `GP_sysmodel`: the vehicle model for simulations

### GP_ARX_simulation
The GP_ARX_simulation folder contains the code for simulating a platoon of vehicles using MPC with an ARX+GP model, where the simulated vehicle is also an ARX+GP model. Specifically, the following files are included:
* `GP_MPC_Platoon_simulation`: the main file for running the simulation
* `GP_MPC_Platoon`: the GP-MPC
* `GP_sysmodel`: the vehicle model for simulations

### GP_training
The GP_training folder contains the code for training GP models. Specifically, the following files are included:
* `G1.mat`: the collected data
* `GP_training`: train the standard or vanilla GP model
* `GP_RE_trainForSpares`: create the sparse GP model
* `plot_to_pdf`: export PDFs of plots

### SparseGP_ARX_simulation
The `SparseGP_ARX_simulation` folder contains two subfolders: `ARX_simulation` and `SparseGP_simulation`. The `ARX_simulation` is similar to the previous one, except for some variable changes. In `SparseGP_simulation`, the MPC uses a sparse GP+ARX model, while the simulated vehicle remains an ARX+GP model.

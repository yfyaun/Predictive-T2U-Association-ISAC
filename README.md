# Predictive Target-to-User Association in Complex Scenarios via Hybrid-Field ISAC Signaling

This repository contains the simulation code for the paper titled "Predictive Target-to-User Association in Complex
Scenarios via Hybrid-Field ISAC Signaling" submitted to IEEE Vehicular Technology Conference 2025.

## Running the Simulation

1. **First run the Monte Carlo simulation**:
   ```
   run('Monte_Carlo_simulation.m')
   ```
   This will perform multiple trials of the simulation and save the results in the `results` folder.

2. **Then visualize the results**:
   ```
   run('result_visualization.m')
   ```
   This will generate figures showing tracking performance, achievable rates, and cumulative distribution functions.

## Directory Structure

- `functions/`: Core implementation of tracking algorithms and simulation components
- `trajectory_data/`: Vehicle trajectory data used in simulations
- `results/`: Output directory for simulation results

## Citation

If you use this simulation code in any way, please cite the paper above.

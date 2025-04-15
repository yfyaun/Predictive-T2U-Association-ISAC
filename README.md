# Predictive Target-to-User Association in Complex Scenarios via Hybrid-Field ISAC Signaling

This repository contains the simulation code for the paper titled "Predictive Target-to-User Association in Complex
Scenarios via Hybrid-Field ISAC Signaling" accepted by IEEE Vehicular Technology Conference 2025.

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
   This will generate figures in the paper, showing tracking performance (Fig. 4), achievable rates (Fig. 5), and cumulative distribution functions (Fig. 6).

## Directory Structure

- `functions/`: Core implementation of tracking algorithms and simulation components
- `trajectory_data/`: Vehicle trajectory data used in simulations
- `results/`: Output directory for simulation results

## Citation

If you use this simulation code in any way, please cite the following paper:
```bibtex
@article{yuan2025predictive,
  title={Predictive Target-to-User Association in Complex Scenarios via Hybrid-Field ISAC Signaling},
  author={Yuan, Yifeng and Wen, Miaowen and Zheng, Xinhu and Wang, Shuoyao and Gao, Shijian},
  journal={arXiv preprint arXiv:2501.10676},
  year={2025}
}

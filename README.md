# Hawkeye: Simulation-Based UAV-Gimbal Visual Tracking

**Weighted multi-criteria NPSO-tuned state-space PD control in Gazebo and ROS 2**

Hawkeye is a research framework for vision-based UAV gimbal target tracking. It combines image-based target observations, an observer-based state-space proportional–derivative (SS-PD) controller, and offline Novel Particle Swarm Optimization (NPSO) for weighted multi-criteria tuning.

> **Release status:** Public repository scaffold. Source packages and experiment logs are **not included in this initial scaffold**. This README must be updated with tested build and execution commands before the repository is described as reproducible.

## Research scope

The reported study evaluates a two-axis yaw–pitch gimbal in **Gazebo Harmonic** with **ROS 2 Humble**. Scenario 1 applies prescribed lateral carrier motion with a stationary target. Scenario 2 evaluates transient recovery from an initial image-plane offset with a stationary carrier and target. These are simulation results, not flight-test validation.

The manuscript currently describes YOLO–ByteTrack detection and tracking. **The exact YOLO model version must be verified against the released implementation** before setting the dependency and model-weight instructions.

## Repository layout

| Directory | Intended contents |
| --- | --- |
| `ros2_ws/src/` | ROS 2 packages and launch files |
| `gazebo/` | Gazebo world, UAV, and yaw–pitch gimbal assets |
| `perception/` | Target detection and association implementation |
| `controllers/` | Baseline PD and observer-based SS-PD code |
| `optimization/` | NPSO tuning and weighted metric evaluation |
| `experiments/` | Scenario and evaluation scripts |
| `results/` | Shareable, selected evaluation results |
| `figures/` | Plotting scripts and publication figures |
| `docs/` | Setup instructions and reproducibility checklist |

## Experimental configuration reported in the paper

- Control sampling period: **1 ms nominal** (the manuscript does not establish measured end-to-end execution timing).
- NPSO parameter vector: proportional gain \(K_P\), derivative gain \(K_D\), observer bandwidth \(\omega_o\).
- NPSO reported configuration: **20 particles**, **11 iterations**; reconcile actual evaluations against the source implementation and Algorithm A1 before claiming an exact evaluation total.
- Parameter bounds: \(K_P\in[7,14]\), \(K_D\in[0,0.70]\), \(\omega_o\in[20,1000]\) rad/s.
- Scenario 1 lateral motion: \(y(t)=0.35\sin(0.5t)\) m (peak speed 0.175 m/s; peak acceleration 0.0875 m/s²).
- Optimization metrics: mean radial error, radial-error volatility, second-order visual-error variation, and image-centre zero crossings.

## Getting started

The original ROS 2 packages, Gazebo simulation assets, controller scripts and optimization files still need to be copied into this repository. After adding the code, document and test the full sequence in [`docs/reproducibility.md`](docs/reproducibility.md).

Do not commit `build/`, `install/`, `log/`, bag recordings, private credentials, or unlicensed model weights. See [`.gitignore`](.gitignore).

## Results and limitations

The manuscript reports a reduction in mean settling time from **2.729 s to 1.387 s** when comparing the tested conventional PD and SS-PD configurations. The equal-weight NPSO configuration yields the lowest second-order visual-error variation and zero-crossing metric among the tested NPSO cases, but not the lowest mean tracking error. Comparisons using different initial image offsets must not be interpreted as controlled head-to-head comparisons.

Simulation-only findings are limited to the evaluated deterministic scenarios. Hardware validation, randomized disturbances, and independent-seed NPSO repeatability remain open tasks.

## Citation

If you use this research, cite the associated *Drones* manuscript once publication details are finalized. Until then, please refer to this repository and its commit SHA.

## License

**No open-source license has been selected yet.** Public visibility does not grant permission to reuse or redistribute code. Add an appropriate license after confirming approval from all code contributors and any institutional requirements.

## Authors

Msuega Jnr Iorpenda, Mario Lazzarini Sola, and Volker Willert — Center for Robotics (CERI), Technical University of Applied Sciences Würzburg–Schweinfurt (THWS).

# Reproducibility checklist

This repository is initially a scaffold. Mark steps complete **only after testing the commands on a clean workstation**.

## Environment

- [ ] Confirm Ubuntu version and architecture
- [ ] Document installation of ROS 2 Humble and Gazebo Harmonic
- [ ] Record Python version and locked dependencies
- [ ] State exact target detection model/version and lawful model-weight source
- [ ] Document ROS 2 workspace package dependencies

## Launch and operation

- [ ] Provide a tested `colcon build` command
- [ ] Provide tested Gazebo launch command
- [ ] Document ROS topics and message types (e.g., gimbal velocity command)
- [ ] Provide tested perception/controller launch sequence
- [ ] Document the gimbal joint limits, velocity saturation, and camera intrinsics from simulation assets
- [ ] Verify measured perception and command publication frequencies

## Optimization

- [ ] Commit source code implementing NPSO
- [ ] Document all initial/pilot and optimizer random seeds
- [ ] Reconcile initial-swarm evaluations vs iteration evaluations
- [ ] Document reference constants for metric normalization and how computed
- [ ] Verify metric definitions match code (especially volatility and second differences)
- [ ] Record experiment weight vectors and optimized gains

## Results

- [ ] Add raw, machine-readable results for all six NPSO experiments
- [ ] Include normalized and raw metrics derived from the same trial windows
- [ ] Add figure-generation scripts with fixed dependencies
- [ ] Clarify which results are in-sample
- [ ] Document limitations of scenario-to-scenario overshoot comparisons
- [ ] Document artifact checksums if distributing external recordings

## Publication

- [ ] Verify consent/licensing for collaborator contributions and model assets
- [ ] Choose and add a LICENSE
- [ ] Add a publication DOI and CITATION.cff when confirmed
- [ ] Remove sensitive paths, tokens, email credentials and private data

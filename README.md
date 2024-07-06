# Torque-Free Motion of a Rigid Body Simulation

## Overview
This Github repo 

## Features
- Visualization of rotations about the maximum, intermediate, and minimum principal axes.
- Simulation of torque-free rotation of prolate and oblate objects.
- Illustration of space and body cones showing the rotational behavior of the body.
- Demonstrate the dual-spin stabilization of a spacecraft with a spinning flying wheel.

## Simulations
There are three independent MATLAB files for the simulations:
1. **Principal Axes Theorem:**
   - Simulates the motion of a rigid body rotating about its principal axes.
   - ![Rotation about principal axes](demo/int_axis_thm.gif)
   - `principal_axes_thm.m`

2. **Prolate vs. Oblate Objects:**
   - Demonstrates the different behaviors of prolate and oblate objects, visualizing the body cone and space cone.
   - ![Prolate vs. Oblate Objects](demo/prolate_vs_oblate.MOV)
   - `prolate_vs_oblate.m`

3. **Dual-Spin Stabilization:**
   - Simulates the stabilization of a dual-spin spacecraft.
   - ![Dual spin motion](demo/dual_spin.gif)
   - `dual_spin_stabilization.m`

## Usage
1. **Clone the repository:**
   ```bash
   git clone https://github.com/yi-hsuan-chen/Torque-Free-Motion-of-a-Rigid-Body-Simulation.git
   ```
2. **Navigate to the project directory:**
   ```bash
   cd Torque-Free-Motion-of-a-Rigid-Body-Simulation
   ```
3. **Open MATLAB and run the desired script:**
  ```matlab
  run('principal_axes_thm.m');
  % run('prolate_vs_oblate.m');
  % run('dual_spin_stabilization.m')
  ```

## Technical Details
For more technical details, please refer to the final project report included in this repository (ENAE646_YiHsuan_Final_Project).

## Development and Testing
These files were developed and tested using MATLAB R2023b.

## Contact
For any questions or further information, please contact: Yi-Hsuan Chen (yhchen91@umd.edu)

## References
1. Kasdin, N. Jeremy, and Derek A. Paley. "Engineering dynamics: a comprehensive introduction." (2011): 1-704.
2. Dual-Spin Spacecraft, Stabilizing Rotation About Any Principal Axis, Dr. Shane Ross. https://www.youtube.com/watch?v=8uOxYf9nLNw

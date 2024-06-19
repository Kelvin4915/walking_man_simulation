# HumanoidBot


https://github.com/Kelvin4915/walking_man_simulation/assets/134540002/b4001b4d-5d59-49c0-bd94-20eace003619


This code aims to simulate and validate the Linear Quadratic Regulator (LQR), an optimal control algorithm.

## Objectives

* Simulate human walking actions.
* Control the motion of the Center of Mass (CoM) using the Center of Pressure (CoP) and a regulator.
* Test the model under both fixed and variable velocities for the Center of Mass (CoM).
* Apply external forces to the system to evaluate the precision and accuracy of the Linear Quadratic Regulator (LQR).

## Assumptions

* The leg lengths are constant, resulting in a consistent g/L ratio.
* The angle between the line joining the Centers of Pressure (CoP) and Center of Mass (CoM) and the vertical axis is very small, allowing the sine of the angle to be approximated by the angle itself.

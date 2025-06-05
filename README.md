# Dynamixel Interface for Tendon-Driven Continuum Robots
This package is a specialized version of the [ros_dynamixel_pkg](https://github.com/Elektron97/ros_dynamixel_pkg) repo for low-level control of tendon-driven continuum robots. In this version, the dynamixels are controlled in number of turns and they publish their current information.

## Torque and Current Relation
```cpp
// current(torque) = coeff_2*torque^2 + coeff_1*torque + coeff_0
#define COEFF_0 0.1327
#define COEFF_1 0.5753
#define COEFF_2 0.2030
```
$$
i(\tau) = coeff_2*\tau^2 + coeff_1*\tau + coeff_0
$$

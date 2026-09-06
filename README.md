# Dynamixel Interface

Low-level driver for a tendon-driven continuum robot built on Robotis XM430-W210-R servos. One node, `motor_io`, talks to all the motors and gives you three ways to drive them. Forked from [ros_dynamixel_pkg](https://github.com/Elektron97/ros_dynamixel_pkg).

## Running it

```bash
roslaunch dynamixel_interface motor_io.launch command_mode:=current
roslaunch dynamixel_interface motor_io.launch command_mode:=turns
roslaunch dynamixel_interface motor_io.launch command_mode:=current_position current_limit:=1.5
```

`command_mode` is fixed for the life of the node — changing it means restarting, since it reprograms the servos' operating-mode register, and that register can't be touched while torque is on.

- **current** — command current [A] on `cmd_currents`, or torque [Nm] on `cmd_torques` (same fit as below). Use this if you're running your own control law — e.g. `τ = PID(position) + gravity_compensation` — and just want to hand the servo a final torque.
- **turns** — command position by turn count on `cmd_turns`, relative to a zero latched at startup (and re-latched every time torque is re-enabled). No torque limit: the servo pushes as hard as it can to get there.
- **current_position** — same as turns, but with a torque ceiling (`current_limit`, amps) set once at startup. This is the one to reach for if a stall or a snag shouldn't be allowed to overtension a tendon. See [Robotis' docs](https://docs.robotis.com/docs/dxl/model_reference/x_series/xm_series/xm430-w210/) on Current-based Position Control for how the servo handles this internally.

Feedback is always both: `read_turns` and `read_currents`, published together at 30 Hz from a single bus read.

`switch_torque` (`std_srvs/SetBool`) turns all motors on or off together; turning them back on re-zeroes the turn count from wherever they physically ended up.

## Torque and current

The servos are current-controlled, so torque commands go through an empirical quadratic fit rather than a datasheet constant:

```cpp
current(|τ|) = coeff_2·|τ|² + coeff_1·|τ| + coeff_0
coeff_0 = 0.1327
coeff_1 = 0.5753
coeff_2 = 0.2030
```

The fit was characterized on magnitudes, so the sign of the input is reapplied afterwards — it stays odd-symmetric around zero rather than always coming out positive.

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS1 (catkin) C++ package that drives Robotis Dynamixel X-series servos (XM430-W210-R) for a
tendon-driven continuum robot. It is a specialized fork of
[ros_dynamixel_pkg](https://github.com/Elektron97/ros_dynamixel_pkg). A single motor-driver class,
`DynamixelInterface`, is configured at construction with a command modality — current (torque)
control, turns (extended-position) control, or current-based-position control (turns with a torque
ceiling) — while feedback (turns + current/torque) is always available regardless of which modality
is active, since present-position and present-current are readable in any operating mode.

## Build

This package lives inside a catkin workspace (`~/catkin_ws`), so build from the workspace root, not
from this package directory:

```bash
cd ~/catkin_ws
catkin_make            # or: catkin build dynamixel_interface
source devel/setup.bash
```

Depends on the `dynamixel_sdk` catkin package (source lives at `~/catkin_ws/src/DynamixelSDK`). This
workspace uses a `CATKIN_WHITELIST_PACKAGES` cache variable to limit which packages `catkin_make`
builds — if `dynamixel_interface` (or `dynamixel_sdk`) isn't in that list, `catkin_make --pkg
dynamixel_interface` silently no-ops. Pass `-DCATKIN_WHITELIST_PACKAGES="<existing list>;dynamixel_interface;dynamixel_sdk"`
to build it without permanently overwriting the cached whitelist for other packages.

There is no test suite and no linter configured in this package.

## Running

```bash
rosrun dynamixel_interface motor_io _command_mode:=current                                # or: turns
rosrun dynamixel_interface motor_io _command_mode:=current_position _current_limit:=1.5   # [A]
```

`command_mode` is a private ROS param read once at startup (default `"current"`; also `"turns"` or
`"current_position"`); switching it requires restarting the node, since it reprograms the servos'
`ADDR_OP_MODE` register, which requires torque to be disabled — there is no runtime mode-switch
service. `current_position` (`CURRENT_POSITION_MODE` = 5) commands turns like `turns` mode but caps
the position controller's torque via `~current_limit` [A], written once at startup as `Goal Current`
(a RAM register — no EEPROM/torque-disable dance needed); see the
[Robotis docs](https://docs.robotis.com/docs/dxl/model_reference/x_series/xm_series/xm430-w210/) for
the mode's control architecture. Leaving `~current_limit` unset in this mode logs a warning and falls
back to `MAX_CURRENT` (no effective limiting). In `turns`/`current_position` modes, `~max_turns`
[turns] caps the magnitude of a commanded turn count (`turns_saturation`, saturating and warning
past it); defaults to `MAX_TURNS` (`dynamixel_utils.h`) if unset. `~profile_velocity`/
`~profile_acceleration` set the raw Profile Velocity/Profile Acceleration register values
(`ADDR_PROFILE_VEL`/`ADDR_PROFILE_ACC`), written once per motor at startup regardless of
`command_mode` — they shape the trapezoidal motion profile used by the position controller (higher
= faster/more abrupt moves); default to `PROFILE_VEL_VALUE`/`PROFILE_ACC_VALUE`
(`dynamixel_utils.h`) if unset. `~node_frequency` [Hz] sets the feedback timer's rate
(`read_turns`/`read_currents`), independent of `command_mode`; defaults to `NODE_FREQUENCY`
(`dynamixel_node.h`, 30 Hz) if unset. This starts the `motor_io` node, which opens `/dev/ttyUSB0` at
115200 baud (see `DEVICE_NAME`/`BAUDRATE` in `dynamixel_utils.h`) and talks to `N_MOTORS` (7) servos
with IDs `1..N_MOTORS`.

ROS interface (namespace `/dynamixels`, defined in `dynamixel_node.h`):
- Subscribes `cmd_currents` [A] (mode `current`) **or** `cmd_turns` (modes `turns`/`current_position`)
  (`std_msgs/Float32MultiArray`, size must equal `N_MOTORS`). In mode `current`, `cmd_torques` [Nm] is
  additionally subscribed — an alternate unit onto the same current command path
  (`DynamixelInterface::set_torques`, via `torque2Current`/`torque2Register`); publishing to both at
  once is undefined (whichever callback lands last wins) so pick one.
- Publishes `read_turns` and `read_currents` (`std_msgs/Float32MultiArray`) at `~node_frequency`
  [Hz] (private ROS param, defaults to `NODE_FREQUENCY` = 30 Hz if unset) — both, unconditionally,
  regardless of `command_mode`. Independent of the command-side params above.
- Service `switch_torque` (`std_srvs/SetBool`) — enables/disables torque on all motors; re-enabling
  also re-latches the "zero turns" reference position (`update_initPos`). `res.success` reflects
  whether every motor actually acknowledged the write, not a hardcoded `true`.

### Keyboard teleop (`scripts/keyboard_teleop.py`)

```bash
roslaunch dynamixel_interface keyboard_teleop.launch                          # auto-detects command_mode
rosrun dynamixel_interface keyboard_teleop.py _command_mode:=turns            # or set it explicitly
```

A standalone Python node (no C++ code involved) publishing hand-driven commands onto whichever
topic `motor_io` is subscribed to. Keys `1`-`7` select one of the `N_MOTORS` (7) motors; `w`/`+` and
`s`/`-` nudge the selected motor's commanded value by `~current_step`/`~turns_step`; `r` zeros the
selected motor; `space` zeros all of them; `q`/Ctrl-C quits, publishing an all-zero command first as
a safety net. Reads raw keypresses via `termios`/`tty` (needs a real interactive stdin — `roslaunch`
doesn't give a node one, so `keyboard_teleop.launch` runs it inside `xterm` via `launch-prefix`;
`rosrun` from a normal terminal works directly).

`~command_mode` is optional: if unset, `detect_command_mode()` queries the ROS master's system state
(`rosgraph.Master.getSystemState()`) for which of `/dynamixels/cmd_currents` /
`/dynamixels/cmd_turns` already has a subscriber (i.e. whichever `motor_io` picked at its own
startup) and mirrors that choice — `turns` and `current_position` are indistinguishable this way
(both subscribe to `cmd_turns`) but are commanded identically, so that's harmless. If neither topic
has a subscriber yet (`motor_io` not up, or master unreachable), it warns and falls back to
`current`. Passing `~command_mode` explicitly skips detection outright. `~current_limit`/
`~turns_limit` apply a local symmetric clamp on every published value, independent of (and in
addition to) the saturation `DynamixelInterface` applies server-side.

## Architecture

### `DynamixelInterface` (`dynamixel_interface.h/.cpp`)

One concrete class (no template, no subclasses) that owns the shared `PortHandler`/`PacketHandler`
and both the position (`ADDR_GOAL_POSITION`/`ADDR_PRESENT_POSITION`, 4 bytes) and current
(`ADDR_GOAL_CURRENT`/`ADDR_PRESENT_CURRENT`, 2 bytes) `GroupSyncWrite`/`GroupSyncRead` objects. Which
write path is active is decided once, at construction, by the `CommandMode` (`CURRENT`, `TURNS`, or
`CURRENT_POSITION`) passed in — that's also what gets programmed into every motor's `ADDR_OP_MODE`
register. `CURRENT_POSITION` (`CURRENT_POSITION_MODE` = 5) is commanded exactly like `TURNS` (same
`set_command` branch — both write goal position), but additionally has a torque ceiling
(`current_limit_amps` constructor arg) latched once, at construction, into `Goal Current` — the
position controller then can't exceed it regardless of what turn count is commanded. The
`profile_velocity`/`profile_acceleration` constructor args (raw register values, defaulting to
`PROFILE_VEL_VALUE`/`PROFILE_ACC_VALUE`) are likewise written once per motor at construction, to
`ADDR_PROFILE_VEL`/`ADDR_PROFILE_ACC` — independent of `mode` (harmless, unused writes in `CURRENT`
mode, since that mode doesn't drive the position controller). Read paths (`get_turns`,
`get_currents`, `get_torques`) are always live regardless of `mode`.

- `set_command(cmd)` — single write entry point; internally dispatches to the current-register write
  (`CURRENT` mode) or the position-register write (`TURNS`/`CURRENT_POSITION`, identical command
  shape) depending on `mode`. Writes go through a preallocated flat byte buffer (member, sized once
  at construction) rather than allocating per call.
- `set_torques(torques)` — torque [Nm] command, only valid in `CURRENT` mode; converts via
  `torque2Current` then shares the exact same register-saturation/write path as `set_command`'s
  `CURRENT` branch (both funnel through the private `commandCurrents()` helper), so amps- and
  Nm-unit commands are saturated identically.
- `get_turns`/`get_currents`/`get_torques`/`get_feedback` — always available; turns are relative to
  `initial_positions`, captured at construction and re-captured by `update_initPos()` whenever
  `enableTorque()` runs. All four are backed by one `feedback_syncRead` spanning present-current,
  -velocity and -position (contiguous in the control table, `FEEDBACK_BYTE_LENGTH` in
  `dynamixel_utils.h`), so a single bus transaction yields every feedback field. Prefer
  `get_feedback(turns, currents)` when both are needed (e.g. `Ros_Dynamixel_Node`'s main loop) —
  calling `get_turns()` then `get_currents()` separately would cost two transactions instead of one.
- `enableTorque()`/`disableTorque()` return `bool` (aggregate success across all motors, continuing
  past any single motor's comm failure rather than aborting the loop) so callers (the `switch_torque`
  service) can report real success/failure.
- `allMotorsReady()` — per-motor bring-up status tracked during construction; a failed init write for
  one motor no longer blocks initialization of the rest.
- The destructor is the safety-critical shutdown path (zero current or return-to-initial-position,
  then `disableTorque()`) and runs whenever a `DynamixelInterface` object is destroyed, not just on
  clean exit. Since there's no template base class involved, this runs with no virtual-dispatch
  pitfalls.

### `dynamixel_utils.h/.cpp`

Free functions doing all register⇄physical-unit conversions (`current2Register`, `register2Turns`,
`register2Torque`, `torque2Current`, `velocity2Register`, saturation helpers, etc.) and every
control-table address/constant — shared by `DynamixelInterface` regardless of mode.

- **Torque↔current mapping** is an empirical quadratic fit characterized on `|torque| -> |current|`
  (`current(|τ|) = COEFF_2·|τ|² + COEFF_1·|τ| + COEFF_0`, sign re-applied afterwards — see
  `README.md` and the `COEFF_*` defines), inverted in `current2Torque` via the quadratic formula. If
  these coefficients are re-calibrated, update both the header defines and the README's copy of the
  same formula.
- `ONE_TURN_REGISTER` = 4096 (XM430-W210 encoder counts per revolution).
- `MAX_TURNS` = 3.0 is the compile-time default turns-saturation limit; overridable per-run via the
  `~max_turns` ROS param (see Running), latched into `DynamixelInterface`'s `max_turns` member at
  construction and passed through to `turns_saturation`.

### `Ros_Dynamixel_Node` (`dynamixel_node.h/.cpp`)

The single ROS node wrapper, replacing the old `current_node`/`ros_utils` split. Reads
`~command_mode` once at construction (before `DynamixelInterface` is built), subscribes to the
matching command topic, and always publishes both feedback topics on a timer whose rate is
`~node_frequency` (default 30 Hz, `NODE_FREQUENCY` in `dynamixel_node.h`).

### Build graph (`CMakeLists.txt`)

`dynamixel_utils` (dynamixel_utils.cpp) → `${PROJECT_NAME}` / `dynamixel_interface`
(dynamixel_interface.cpp, links `dynamixel_utils`) → `dynamixel_node` (dynamixel_node.cpp, links
`${PROJECT_NAME}`) → `motor_io` executable (links `dynamixel_node`). The `${PROJECT_NAME}` library
target matches what `catkin_package()` exports, so downstream packages depending on
`dynamixel_interface` resolve correctly.

### History

Earlier revisions of this package had two hand-forked class hierarchies —
`Dynamixel_Motors<T>`/`Current_Dynamixel` (current control) and `Dynamixel_Motors<T>`/
`ExtPos_Dynamixel` (turns control) — that only differed in register byte width, plus two competing
`Ros_Dynamixel_Node` node wrappers of which only one was ever linked into `motor_io`. These were
unified into the single `DynamixelInterface`/`Ros_Dynamixel_Node` above; per-motor selective
enable/disable (`set_turns_disable`/`motors_mask`, previously dead code) was intentionally dropped in
the unification in favor of the single all-motors `switch_torque` service.

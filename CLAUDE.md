# CLAUDE.md - PyBullet UR5 + Robotiq Gripper Simulation

## Project overview

This repository contains a PyBullet simulation built around a UR5 arm with a
Robotiq gripper and a fixed `skew-box-button` object.

What is implemented in the current repository:

- A base PyBullet environment in `env.py`
- UR5 + Robotiq 85 / 140 and Panda robot classes in `robot.py`
- A slider-driven demo in `main.py`
- A keyboard-driven force feedback demo in `forcelevel_simulation.py`
- Experiment 1 in `experiment1_contact_judgment.py`
- ESP32 MicroPython firmware in `sevropwm_simulation.py`

What is not implemented yet:

- Experiment 2 force-maintenance code
- `experiment2_config.yaml`
- `experiment2_force_maintenance.py`
- `plot_experiment2_results.py`

The detailed Experiment 2 specification lives in `需求.md`, but that experiment
is still a plan, not a checked-in program.

## File map

| File | Role |
|------|------|
| `env.py` | Defines `ClutteredPushGrasp`. Connects to PyBullet, loads the plane, robot, and `urdf/skew-box-button.urdf`, creates GUI sliders, and runs 120 simulation substeps inside `step()`. |
| `robot.py` | Defines `UR5Robotiq85`, `UR5Robotiq140`, and `Panda`. End-effector IK is handled in `move_ee()`. Robotiq grippers are controlled through mimic joints and gear constraints. |
| `utilities.py` | Defines `Models`, `YCBModels`, and `Camera`. `YCBModels` exists, but the current environment does not actually spawn YCB objects from it. |
| `agent.py` | Minimal `BaseAgent` and `RandomAgent` stubs. Not used by the demos or experiments in this repository. |
| `main.py` | Minimal demo loop that reads GUI sliders and applies them to the robot through `env.step(...)`. |
| `forcelevel_simulation.py` | Keyboard-controlled demo for fingertip force sensing and ESP32 feedback. Includes `create_environment()`, `read_separate_fingertip_forces()`, `force_to_level()`, `generate_packet()`, and `ESP32FeedbackClient`. |
| `experiment1_contact_judgment.py` | Experiment 1 implementation. Adds `TrialRecorder`, condition switching, trial triggering with SPACE, and CSV logging. |
| `sevropwm_simulation.py` | ESP32 MicroPython TCP server that parses the feedback packet and drives two PWM servo channels. Runs on the ESP32, not on the PC. |
| `需求.md` | Experiment 2 requirements and design notes. This file is a spec, not code. |
| `urdf/` | Robot and object URDF files. |
| `meshes/` | Mesh assets used by the URDFs. |

## Current architecture

### Environment lifecycle

```python
from env import ClutteredPushGrasp
from robot import UR5Robotiq85
from utilities import YCBModels

robot = UR5Robotiq85((0, 0.5, 0), (0, 0, 0))
ycb_models = YCBModels("./data/ycb/**/textured-decmp.obj")
env = ClutteredPushGrasp(robot, ycb_models, camera=None, vis=True)
env.reset()
# ... use env ...
env.close()
```

Notes:

- `ClutteredPushGrasp.__init__()` connects to PyBullet and loads the scene.
- `env.robot.id` and `env.boxID` are the main body IDs used by the scripts.
- The `models` argument is accepted by `ClutteredPushGrasp`, but the current
  implementation does not call `models.load_objects()` or add YCB objects.

### Robot control

End-effector control uses:

```python
action = np.array([x, y, z, roll, pitch, yaw, gripper_opening], dtype=float)
env.robot.move_ee(action[:6], control_method="end")
env.robot.move_gripper(action[6])
```

After sending commands, the code must advance physics with
`p.stepSimulation()` or `env.step_simulation()`.

### Gripper kinematics

`UR5Robotiq85.move_gripper()` converts opening length in metres into the mimic
parent joint angle:

```python
open_angle = 0.715 - math.asin((open_length - 0.010) / 0.1143)
```

Important facts from the current code:

- `gripper_range = [0, 0.085]`
- The scripts currently clamp gripper commands to `[0.0, 0.085]`
- Any narrower "safe operating range" is a usage guideline only, not a code
  constraint in the current repository

### Fingertip contact force reading

The two experiment scripts use these constants:

| Constant | Index | Link name |
|----------|-------|-----------|
| `TF_LINK` | 12 | `left_inner_finger_pad` |
| `IF_LINK` | 17 | `right_inner_finger_pad` |

`read_separate_fingertip_forces(env)` calls:

```python
p.getContactPoints(bodyA=env.robot.id, bodyB=env.boxID)
```

It filters by `contact[3]` for the robot link index, reads `contact[9]` as the
normal force, and returns:

```python
(
    tf_force_sum, tf_max_force, tf_contact_count,
    if_force_sum, if_max_force, if_contact_count,
)
```

Force smoothing in both experiment scripts is an exponential moving average:

```python
smooth = (1.0 - alpha) * smooth + alpha * raw
```

The current force-to-level thresholds are:

- `< 5.0` -> level `0`
- `< 50.0` -> level `1`
- `< 100.0` -> level `2`
- otherwise -> level `3`

### ESP32 TCP protocol

Packet format:

```text
[AA 55 AA 55] [payload_len] [payload...] [checksum]
```

Payload format:

```text
[1, TF_level, 2, IF_level]
```

`ESP32FeedbackClient` in `forcelevel_simulation.py` and
`experiment1_contact_judgment.py` currently does the following:

- Connects to `192.168.1.16:12345`
- Deduplicates packets by comparing against `self.last_payload`
- On send failure, sets `self.sock = None` and reconnects on a later send
- `close()` only closes the socket

Important detail:

- The scripts themselves send `(0, 0)` in their `finally` blocks before calling
  `close()`

Do not change the packet format unless you also update
`sevropwm_simulation.py`.

## Script behavior

### `main.py`

- Uses GUI sliders created by `env.py`
- Does not implement the keyboard control scheme from the experiment scripts

### `forcelevel_simulation.py`

Keyboard controls:

| Key | Action |
|-----|--------|
| `I` / `K` | `x +` / `x -` |
| `J` / `L` | `y -` / `y +` |
| `U` / `O` | `z +` / `z -` |
| `1` / `2` | `roll -` / `roll +` |
| `3` / `4` | `pitch -` / `pitch +` |
| `5` / `6` | `yaw -` / `yaw +` |
| `Z` / `X` | close / open gripper |
| `P` | print current contacts |
| `Q` | quit |

### `experiment1_contact_judgment.py`

This script uses the same motion keys as `forcelevel_simulation.py`, and adds:

| Key | Action |
|-----|--------|
| `SPACE` | trigger a trial |
| `C` | toggle `visual_only` / `vision_haptic` |
| `P` | print current contacts |
| `Q` | quit |

When a trial is triggered, the script blocks on terminal input and asks the
experimenter to enter `1` or `0`.

### Simulation stepping helper

Both experiment scripts define `safe_step_simulation(env)`:

- Use `env.step_simulation()` if present
- Otherwise fall back to `p.stepSimulation()`

In `ClutteredPushGrasp`, `step_simulation()` also sleeps for `1 / 240` seconds
when `vis=True`.

## Experiment 2 status

Experiment 2 is not implemented in this repository yet.

`需求.md` describes the intended design, including ideas such as:

- config-driven parameters
- a force-maintenance state machine
- auto perturbation of the gripper
- CSV logging
- ESP32 feedback integration

Those are requirements and design notes only. They are not current code.

## Dependencies in the current codebase

Directly imported by the checked-in Python files:

```text
pybullet
pybullet_data
numpy
opencv-python
scipy
attrdict
tqdm
matplotlib
```

Imported optionally:

```text
torch
```

Notes:

- `pyyaml` is not used by the current checked-in code.
- `main.py`, `forcelevel_simulation.py`, and
  `experiment1_contact_judgment.py` all construct a `./data/ycb/...` glob, but
  this repository currently does not include a `data/ycb` directory.

## Running

Current checked-in entry points:

```bash
python main.py
python forcelevel_simulation.py
python experiment1_contact_judgment.py P01 visual_only
python experiment1_contact_judgment.py P01 vision_haptic
```

Files mentioned in `需求.md` for Experiment 2 are not available to run yet.

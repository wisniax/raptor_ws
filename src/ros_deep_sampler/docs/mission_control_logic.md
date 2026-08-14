# Mission Control — Operation and Control Logic

## 1. Overview

The mission controller combines two independent inputs:

1. **Mission command** — what the operator wants the mission to do:
   - `START`
   - `STOP`
   - `ABORT`
   - `RESTART`
   - `CALIBRATE`

2. **Rover control mode** — who controls the sampler:
   - `MANUAL`
   - `AUTONOMY`
   - `NO_SAMPLER`

The general hierarchy is:

```text
Rover Control Mode
        |
        +-- NO_SAMPLER -> STOP
        |
        +-- MANUAL     -> Operator commands
        |
        +-- AUTONOMY   -> Autonomy FSM
```

Mission commands have priority over normal execution.

---

## 2. Mission Commands

```cpp
uint8 START = 0
uint8 STOP = 1
uint8 ABORT = 2
uint8 RESTART = 3
uint8 CALIBRATE = 4
```

| Command | Meaning |
|---|---|
| `START` | Start/continue operation according to the current control mode |
| `STOP` | Stop current motion/mission |
| `ABORT` | Stop and terminate the current autonomy mission |
| `RESTART` | Reset the autonomy FSM and start again from `IDLE` |
| `CALIBRATE` | Execute sampler calibration |

---

## 4. Overall Priority

The mission controller evaluates commands in the following order:

```text
1. Check control mode
       |
       +-- NO_SAMPLER -> STOP
       |
2. Check mission command
       |
       +-- ABORT
       +-- STOP
       +-- RESTART
       +-- CALIBRATE
       +-- START
              |
3. Select control mode
       |
       +-- MANUAL   -> executeManual()
       |
       +-- AUTONOMY -> executeAutonomy()
```

`START` does not determine whether the sampler is manual or autonomous. The rover control mode determines this.

---

## 9. RESTART

When the mission command is `RESTART`:

```cpp
restartAutonomy();
```

The autonomy controller is reset:

```cpp
autonomy_.resetStateVariables();
autonomy_.setState(
    AutonomyController::AutonomyStates::IDLE);
```

The mission command is then changed to `START`.

```text
RESTART
   |
   v
Reset autonomy variables
   |
   v
Set FSM = IDLE
   |
   v
Set command = START
   |
   v
Execute according to current control mode
```

---

## 10. CALIBRATE

When the mission command is `CALIBRATE`:

```cpp
calibrateSampler();
```

The calibration is executed by `AutonomyController`.

After calibration is completed, the mission command is changed to `STOP` and autonomy variables are reset.

```text
CALIBRATE
    |
    v
requestCalibration()
    |
    v
Calibration
    |
    v
Calibration complete
    |
    v
STOP
```

---

## 12. Manual Operation

When:

```cpp
ctrlType_ == ControlType::MANUAL
```

the controller executes:

```cpp
executeManual();
```

Manual execution:

1. Stops autonomy control.
2. Processes new operator commands.
3. Sends the specified commands to `JointMovement`.

```text
START
  |
  v
MANUAL mode
  |
  v
executeManual()
  |
  +-- stop autonomy
  |
  +-- process operator command
          |
          v
     JointMovement
```

Only actuators for which a new command was received should be commanded.

---

## 17. Autonomy Operation

When:

```cpp
ctrlType_ == ControlType::AUTONOMY
```

the controller executes:

```cpp
executeAutonomy();
```

which calls:

```cpp
autonomy_.executeAutonomy(
    sampler_state_,
    *missionCmdMsg,
    *missionFeedbackMsg,
    *joints_,
    this->get_logger());
```

The `MissionControl` node provides the external data and interfaces, while `AutonomyController` executes the autonomy state machine.

---

## 18. Autonomy State Machine

`AutonomyController` contains the states responsible for executing the autonomous sampler mission:

```text
CALIBRATE_PLATFORM
CALIBRATE_DRILL
CALIBRATE_CONTAINER

MOVE_PLATFORM_DOWN
GET_SURFACE_SAMPLE
DRILLING
RECOVER_DRILL

MOVE_DRILL_UP
MOVE_PLATFORM_UP

MOVE_CONTAINER
MOVE_DRILL_CLOSER
PUT_DEEP_SAMPLE
HIDE_DRILL

MOVE_PLATFORM_CLOSER
PUT_SURFACE_SAMPLE
MOVE_PLATFORM_BACK
HIDE_CONTAINER

MEASURE_SAMPLES
DONE
```

`MissionControl` controls the **mission-level operation**, while `AutonomyController` controls the **individual autonomy states**.

---

## 19. Control Mode Transition

### AUTONOMY → MANUAL

```text
AUTONOMY
   |
   v
Stop autonomy
   |
   v
MANUAL
   |
   v
Operator commands
```

The autonomy controller must no longer control the joints after manual control becomes active.

### MANUAL → AUTONOMY

```text
MANUAL
   |
   v
AUTONOMY
   |
   v
Autonomy FSM takes control
```

The autonomy controller should be in a well-defined state before starting execution.

---

## 21. Recommended Control Rules

| Control mode | Mission command | Result |
|---|---|---|
| `NO_SAMPLER` | Any | Stop sampler |
| `MANUAL` | `START` | Enable manual control |
| `MANUAL` | `STOP` | Stop sampler |
| `MANUAL` | `ABORT` | Stop/terminate operation |
| `MANUAL` | `RESTART` | Reset autonomy |
| `MANUAL` | `CALIBRATE` | Run calibration |
| `AUTONOMY` | `START` | Execute autonomy FSM |
| `AUTONOMY` | `STOP` | Stop autonomy |
| `AUTONOMY` | `ABORT` | Stop and terminate autonomy |
| `AUTONOMY` | `RESTART` | Reset FSM to `IDLE` |
| `AUTONOMY` | `CALIBRATE` | Run calibration |

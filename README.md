```markdown
# MAINHORS26 — HORS 2026 Robot Code

This repository contains the Java code for the HORS 2026 FTC robot (MAINHORS26). The project includes both TeleOp and Autonomous modes. The autonomous routines use Pedro pathing (via the "PedroPathing" follower) and an explicit finite-state machine (FSM) for sequencing shooter/intake/claw actions.

This README gives a high-level overview of the code, how TeleOp and Auto are structured, how Pedro pathing is used, and how the Auto state machine works. It also highlights where to change important tuning constants and how to add/adjust path waypoints.

---

## Table of Contents

- [Repository layout](#repository-layout)  
- [Getting started / build & deploy](#getting-started--build--deploy)  
- [TeleOp overview](#teleop-overview)  
- [Autonomous overview](#autonomous-overview)  
  - [ExperimentalPedroAuto (overview)](#experimentalpedroauto-overview)  
  - [Auto state machine (AutoState)](#auto-state-machine-autostate)  
- [Pedro pathing details](#pedro-pathing-details)  
  - [Stacking points / multiple segments in a PathChain](#stacking-points--multiple-segments-in-a-pathchain)  
  - [No-deceleration and velocity constraints](#no-deceleration-and-velocity-constraints)  
- [Tuning and common edits](#tuning-and-common-edits)  
- [Contributing / contacts](#contributing--contacts)  
- [License](#license)

---

## Repository layout

(Only the most relevant folders/files called out here.)

- TeamCode/src/main/java/org/firstinspires/ftc/teamcode/  
  - autopaths/ExperimentalPedroAuto.java — Main autonomous OpMode that uses Pedro pathing and an FSM to sequence intake/claw/shooter actions.  
  - pedroPathing/Constants.java — helper used to construct the Pedro follower (hardware mapping + config).  
  - subsystems/ — Flywheel, TurretController, and other subsystem classes used by TeleOp and Autonomous.  
  - teleop/ — TeleOp OpModes (not enumerated here) used for driver control.

---

## Getting started / build & deploy

1. Install the standard FTC SDK + Android Studio toolchain and set up the project workspace for your robot as usual.
2. Import this repo (or add it to your FTC project) and verify gradle builds in Android Studio.
3. Deploy to your robot controller phone by building and installing the APK from Android Studio.
4. Use the Driver Station phone to select and run TeleOp or Autonomous OpModes.

---

## TeleOp overview

The TeleOp code in this project controls the drivetrain and the robot subsystems used in matches:

- Drivetrain (mecanum / tank / custom as implemented in drive classes).
- Flywheel (shooter) — closed-loop RPM control via `Flywheel` subsystem.
- Turret — controlled by `TurretController` which uses turret encoder and IMU for references.
- Intake motor and compression servos — start/stop intake and move compression servos.
- Claw servo — open/close to pick/place game elements.

Important notes:
- TeleOp sets up and configures hardware names used across both TeleOp and Autonomous (for example motor and servo names like `shooter`, `turret`, `intakeMotor`, `clawServo`, etc.). Make sure your Robot Configuration (on the Robot Controller phone) uses the same names.
- Subsystems expose helper methods for enabling/disabling motors and controlling servo positions. The autonomous code reuses the same subsystems when possible.

---

## Autonomous overview

### ExperimentalPedroAuto (overview)
`ExperimentalPedroAuto` is the OpMode that runs an 11-path sequence built using Pedro pathing. The OpMode:

- Constructs a Pedro `Follower` and builds 11 `PathChain` objects in an inner `Paths` class.
- Uses a finite-state machine (FSM) to coordinate shooter warm-up, path following, and pre-shoot/intake/claw sequences.
- Integrates the existing `Flywheel` (shooter) and `TurretController` subsystems so the turret can track while the robot moves and the flywheel runs closed-loop.

Behavior summary:
- On `start()`: the flywheel is enabled and set to a close-range RPM target. The OpMode waits (with a timeout) until the flywheel is on target, then starts Path 1.
- The robot runs the chain of paths 1..11 in order, with path-specific intake behavior:
  - Some paths start continuous intake for the duration of the path (e.g., path 3, 6, 9).
  - Some paths start a short timed intake when they begin (e.g., paths 4, 7, 10). (This is implemented per-path in `startPath()`.)
- Certain paths (1, 4, 7, 10) end at the "shoot pose" (x=48, y=96). When a path finishes at that pose, the FSM enters a PRE_ACTION state that:
  - Waits for the robot to reach the shoot pose (with a small tolerance), or falls back after a short timeout.
  - Waits an additional PRE_ACTION delay, then runs an intake period, then a short claw close/open sequence, then continues to the next path.

### Auto state machine (AutoState)

The FSM used in Autonomous is explicit and easy to follow. The high-level states are:

- IDLE — not running paths
- WAIT_FOR_SHOOTER — wait until shooter is at target RPM or timeout
- RUNNING_PATH — follower is executing a PathChain
- PRE_ACTION — special entry at paths that end at shoot pose; waits for pose then delays before intake
- INTAKE_RUN — run timed intake for a configured duration, then proceed to claw action
- CLAW_ACTION — close the claw for a short duration, then open and continue
- FINISHED — autonomous sequence complete; hardware remains in last set state (or stopped on `stop()`)

All path-specific intake/timed-intake behavior and the "ends at shoot pose" mapping are implemented in `ExperimentalPedroAuto.startPath(...)`, `endsAtShoot(...)`, and the FSM switch inside `runStateMachine(...)`. If you rename or re-index paths, be careful to keep `endsAtShoot()` aligned with the indices that should trigger PRE_ACTION behavior.

---

## Pedro pathing details

Pedro pathing uses a `Follower` and `PathChain` objects. In this code you will see:

- Building a path:
  - follower.pathBuilder().addPath(new BezierLine(startPose, endPose)).setLinearHeadingInterpolation(...).build()
- Chaining multiple Bezier segments in a single PathChain:
  - You can call `.addPath(...)` multiple times on the same builder; it creates a single PathChain made up of consecutive Bezier segments.
- Following:
  - follower.followPath(paths.PathN);

### Stacking points / multiple segments in a PathChain
- Yes — you can "stack" segments by calling `.addPath(...).addPath(...).build()` to create a PathChain that visits intermediate waypoints in order. This is the simplest way to add intermediate points while preserving the same path index and all FSM logic.
- Example:
  ```java
  .pathBuilder()
    .addPath(new BezierLine(new Pose(15,57), new Pose(30,57)))
    .addPath(new BezierLine(new Pose(30,57), new Pose(48,96)))
    .setLinearHeadingInterpolation(...)
    .build();
  ```
  This keeps the entire motion under one path index, so FSM logic like "this path ends at shoot pose" stays unchanged.

### No-deceleration and velocity constraints
- PathChain-level methods:
  - `.setVelocityConstraint(value)` — sets a velocity constraint for the entire PathChain.
  - `.setNoDeceleration()` — disables deceleration for the entire PathChain (useful if you want the robot to not slow near the chain end; but it can overshoot and become less accurate).
- Important: these settings apply to the whole PathChain, not to individual segments inside the chain. If you need different constraints or no-deceleration on only one short segment, you must create that segment as a separate PathChain and arrange your sequence accordingly (which may require changing indices or how the FSM triggers PRE_ACTION, etc.).

---

## Tuning and common edits

- Tuning constants in `ExperimentalPedroAuto`:
  - INTAKE_RUN_SECONDS — how long the timed intake runs during PRE_ACTION sequence.
  - TIMED_INTAKE_SECONDS — how long the short timed-intake runs at path start (paths 4/7/10).
  - CLAW_CLOSE_MS — how long to hold claw closed.
  - PRE_ACTION_WAIT_SECONDS and PRE_ACTION_MAX_POSE_WAIT_SECONDS — timing for the PRE_ACTION pose wait/fallback.
  - START_POSE_TOLERANCE_IN — tolerance to determine if robot is at the shoot pose (in inches).
  - AUTO_SHOOTER_RPM and SHOOTER_WAIT_TIMEOUT_MS — shooter target and warmup timeout.
- Path tuning:
  - Use `.setVelocityConstraint(...)` to slow or speed particular PathChains.
  - Use `.setNoDeceleration()` if you need the robot to maintain speed through the segment endpoint (be careful: accuracy may be reduced).
  - Use `.setLinearHeadingInterpolation(startHeadingRad, endHeadingRad)` to control heading interpolation across a chain.
- If you add new intermediate PathChains (split a Path into 2 chains), remember to update:
  - Any index-based logic such as `endsAtShoot(int pathIndex)`.
  - Any intake segment tracking that depends on numeric indices (e.g., `intakeSegmentEnd`).
  - Timed intake behavior which is triggered in `startPath(int idx)`.

---

## Example: adding an intermediate point
Two common approaches:

1. Stack segments inside the same PathChain (preferred if you want no change to FSM):
   - Add two `.addPath(...)` calls on the same builder and `build()`. All path-level settings (velocity, no-decel, heading interpolation) apply to the whole chain.

2. Create a short, separate PathChain for the hop (for example to apply `.setNoDeceleration()` only to that hop):
   - Make a new PathChain (e.g., `Path7_short`) and call `.setNoDeceleration()` on it.
   - Start this new path before the next main path (e.g., call `startPath()` for the short hop, then start the next Path index).
   - This requires adjusting how indices map to behaviors (e.g., ensuring the path that ends at the shoot pose still has the same index in `endsAtShoot` or updating `endsAtShoot` accordingly).

---

## Contributing / contacts

- If you make changes that affect path indices or FSM logic, document them in the code and update this README where appropriate.
- For questions about pathing, FSM behavior, or tuning: open an issue in this repo or contact the repository owner / team lead.

---

## License

This repository does not include a LICENSE file. Add a license file (e.g., MIT) if you intend to make the code open-source and allow reuse.

---
```

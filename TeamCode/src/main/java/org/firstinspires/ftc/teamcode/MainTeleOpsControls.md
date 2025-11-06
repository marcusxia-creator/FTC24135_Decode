# MainTeleOps Control Manual

This document outlines the controls for the `MainTeleOps` op mode.

## Gamepad 1 (Driver)

| Button(s) | Action |
| --- | --- |
| Left Stick | Drivetrain Movement (Strafe/Forward/Backward)|
| Right Stick | Drivetrain Rotation |
| **Back + Left Bumper** | Toggle Alliance (Red/Blue) |
| **DPAD Down** | Reset Odometry to (0, 0, 0) |
| **Start + Left Bumper**| Toggle Control Mode (RUN/TEST) |
| **DPAD Left** | Start ball intake |
| **DPAD Right** | Unjam intake |

## Gamepad 2 (Operator)

| Button(s) | Action |
| --- | --- |
| **A** | Start ball intake sequence. |
| **X** | Start ball offtake (shooting) sequence. |
| **B** | Set the ball handling system to IDLE. |
| **Y** | Start offtake/shooting sequence |
| **DPAD Up** | Toggle between using the AprilTag sequence and manual sequence for offtake. |
| **Right Bumper** | Reset offtake cycle |

## Automatic Actions

- **After 100 seconds:** The robot will automatically switch to using the AprilTag-detected sequence for shooting.

## Intake Process Flow

The intake process is a state machine that handles sweeping, detecting, and storing balls in the spindexer.

1.  **Initiation (Ready)**
    *   Pressing **DPAD Left** on Gamepad 1 starts the intake rollers.
    *   The spindexer automatically positions an empty slot to receive a ball.

2.  **Sweeping & Detection**
    *   The rollers pull a ball into the robot.
    *   An internal sensor detects when the ball has passed the intake and closes the gates to prevent it from falling out.

3.  **Color Analysis & Indexing**
    *   A color sensor determines the ball's color.
    *   The ball's color and position are registered in the system.
    *   The spindexer rotates to the next empty slot, and the gates re-open, ready for the next ball.

4.  **Loop or Full**
    *   The process repeats from the **Sweeping & Detection** stage.
    *   When all spindexer slots are full, the intake motor automatically stops.

5.  **Jam Handling**
    *   **Automatic:** If the system detects the intake motor is struggling, it will automatically reverse for a short pulse to try and clear the jam.
    *   **Manual:** The driver can press **DPAD Right** on Gamepad 1 to trigger a manual un-jamming sequence. This reverses the intake and repositions the spindexer to help clear a stuck ball.

## Offtake (Shooting) Process Flow

The offtake process is a state machine that handles aiming, shooting, and ejecting balls.

1.  **Initiation (IDLE)**
    *   Pressing **Y** on Gamepad 2 starts the shooting sequence.
    *   The system determines if it should use a pre-defined color sequence (from AprilTags) or simply shoot all available balls in reverse order of intake.

2.  **Aiming**
    *   The spindexer rotates to bring the target ball into position for the shooter.

3.  **Shooting**
    *   The shooter motor spins up. The power is calculated based on the robot's distance to the goal.
    *   The gates open to allow the ball to pass.
    *   The ramp servo pushes the ball into the shooter wheel.

4.  **Ejecting**
    *   The system waits for a brief moment to ensure the ball has been fully ejected.
    *   The ball's slot in the spindexer is now marked as empty.

5.  **Loop or Complete**
    *   If there are more balls to shoot in the sequence, the process returns to the **Aiming** state for the next ball.
    *   Once all designated balls have been shot, the system enters a **DONE** state.

6.  **Reset**
    *   From the **DONE** state, pressing the **Right Bumper** on Gamepad 2 resets the entire offtake system, returning it to the **IDLE** state, ready for a new shooting command.

## Target Sequence Selection

`MainTeleOps` manages which shooting sequence is sent to the `OffTakeBall` subsystem. This allows for both manual and autonomous (AprilTag-based) control over the shooting order.

1.  **Sequence Storage in `MainTeleOps`**:
    *   `MainTeleOps` maintains two color sequences: `manualSequence` and `aprilTagSequence`.
    *   A boolean flag, `useAprilTagSequence`, determines which of these two is active.
    *   The operator can toggle this flag using **DPAD Up** on Gamepad 2.
    *   Additionally, the system automatically switches to the AprilTag sequence after 100 seconds of match time.

2.  **Passing the Sequence to `OffTakeBall`**:
    *   In every loop of `MainTeleOps`, the active sequence (either `manualSequence` or `aprilTagSequence`) is passed to the `OffTakeBall` instance using its `setSequence()` method.

3.  **How `OffTakeBall` Uses the Sequence**:
    *   When the shooting process is initiated (with the **Y** button), `OffTakeBall` checks the sequence it has received.
    *   If the sequence contains specific colors (i.e., it's not all `UNKNOWN`), it will enter a color-matching mode and attempt to shoot balls in the specified order. It pre-computes the physical slot order required to match the color sequence.
    *   If the sequence is empty or all `UNKNOWN` (the default for the manual sequence), it will simply shoot all the balls it currently holds in the reverse order they were collected.

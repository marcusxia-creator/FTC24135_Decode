# AutoShooter Class - Usage Guide

## Overview
`AutoShooter` is a RoadRunner 1.0-compatible class that shoots balls according to an AprilTag color sequence. It integrates seamlessly with your autonomous routines and provides real-time telemetry feedback.

## Features
- ✅ **Sequential shooting** based on AprilTag color sequence
- ✅ **Automatic fallback** - shoots remaining balls after sequence
- ✅ **RoadRunner 1.0 Action** integration
- ✅ **FSM-based** state machine for reliable operation
- ✅ **Telemetry feedback** via FTC Dashboard
- ✅ **Blocking mode** available for simple use cases

## Constructor

```java
AutoShooter(RobotHardware robot, List<BallSlot> ballSlots, BallColor[] desiredSequence)
```

**Parameters:**
- `robot` - Your RobotHardware instance
- `ballSlots` - List of BallSlot objects from your spindexer
- `desiredSequence` - Color array from AprilTag (can be null/empty)

## Usage in Auto OpMode

### Method 1: RoadRunner Action (Recommended)

```java
@Autonomous(name = "Shoot Balls Auto")
public class ShootBallsAuto extends LinearOpMode {
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        RobotHardware robot = new RobotHardware(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        
        // Setup spindexer slots
        double[] slotAngles = {0.15, 0.45, 0.75};
        SlotList sharedBalls = new SlotList(slotAngles);
        List<BallSlot> ballSlots = sharedBalls.getBalls();
        
        // Get AprilTag sequence
        AprilTagUpdate aprilTagUpdate = new AprilTagUpdate(hardwareMap);
        waitForStart();
        
        aprilTagUpdate.update();
        SharedColorSequence.aprilTagSequence = aprilTagUpdate.getSequence();
        
        // Build trajectory to shooting position
        Action moveToShoot = drive.actionBuilder(startPose)
                .lineToX(10)
                .turnTo(Math.toRadians(45))
                .build();
        
        // Create shooter with detected sequence
        AutoShooter shooter = new AutoShooter(
            robot, 
            ballSlots, 
            SharedColorSequence.aprilTagSequence
        );
        
        // Execute: drive to position, then shoot
        Actions.runBlocking(
            new SequentialAction(
                moveToShoot,
                shooter.shootAction()
            )
        );
    }
}
```

### Method 2: Parallel with Other Actions

```java
// Shoot while driving to next position
Action driveToNext = drive.actionBuilder(shootPose)
        .strafeTo(new Vector2d(48, 48))
        .build();

AutoShooter shooter = new AutoShooter(robot, ballSlots, sequence);

Actions.runBlocking(
    new ParallelAction(
        shooter.shootAction(),
        driveToNext  // Start driving after shooter spins up
    )
);
```

### Method 3: Blocking Mode (Simple)

```java
// For simple autonomous without RoadRunner Actions
AutoShooter shooter = new AutoShooter(robot, ballSlots, sequence);
shooter.shootBlocking();  // Blocks until complete
```

## Timing Parameters

You can adjust these constants in the `AutoShooter` class:

```java
private static final double SPIN_UP_TIME = 0.5;      // Shooter spin-up time
private static final double FIRE_DELAY = 0.35;       // Time between shots
private static final double GATE_OPEN_DELAY = 0.1;   // Gate opening delay
```

## State Machine Flow

```
SPIN_UP (0.5s)
    ↓
    Opens gates
    ↓
FIRE_SEQUENCE
    ↓ For each color in sequence
    Find matching ball → Fire slot → Wait → Next
    ↓
FIRE_REMAINING
    ↓ For each unfired slot
    Fire if has ball → Wait → Next
    ↓
COMPLETE
    ↓ (0.35s)
    Shutdown shooter & close gates
```

## Telemetry Output

The shooter provides real-time telemetry through FTC Dashboard:

```
Shooter: Spinning up...
Shooter: Ready to fire sequence
Fired: GREEN from slot 0
Sequence Progress: 1/3
Fired: PURPLE from slot 2
Sequence Progress: 2/3
Skipped: GREEN (not found)
Sequence Progress: 3/3
Shooter: Sequence complete, firing remaining
Fired Extra: Slot 1
Remaining Progress: 3/3
Shooter: All balls fired
Shooter: Complete - motors stopped
```

## Example: Full Auto Sequence

```java
@Autonomous(name = "Complete Auto")
public class CompleteAuto extends LinearOpMode {
    
    @Override
    public void runOpMode() throws InterruptedException {
        // === SETUP ===
        Pose2d startPose = new Pose2d(64, 8, Math.toRadians(-30));
        RobotHardware robot = new RobotHardware(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        
        double[] slotAngles = {0.15, 0.45, 0.75};
        SlotList sharedBalls = new SlotList(slotAngles);
        List<BallSlot> ballSlots = sharedBalls.getBalls();
        
        AutoIntake intake = new AutoIntake(robot, ballSlots, slotAngles);
        AprilTagUpdate aprilTagUpdate = new AprilTagUpdate(hardwareMap);
        
        // === TRAJECTORIES ===
        Action moveToIntake = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(32, 32))
                .turnTo(Math.toRadians(90))
                .build();
        
        Action moveToShoot = drive.actionBuilder(new Pose2d(32, 36, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, 0))
                .turnTo(Math.toRadians(45))
                .build();
        
        waitForStart();
        
        // === DETECT SEQUENCE ===
        aprilTagUpdate.update();
        SharedColorSequence.aprilTagSequence = aprilTagUpdate.getSequence();
        
        // === EXECUTE ===
        // 1. Drive to intake area
        Actions.runBlocking(moveToIntake);
        
        // 2. Collect 3 balls
        Actions.runBlocking(intake.collectBallsAction(3));
        
        // 3. Drive to shooting position
        Actions.runBlocking(moveToShoot);
        
        // 4. Shoot in correct order
        AutoShooter shooter = new AutoShooter(
            robot, 
            ballSlots, 
            SharedColorSequence.aprilTagSequence
        );
        Actions.runBlocking(shooter.shootAction());
        
        telemetry.addLine("✅ Auto Complete!");
        telemetry.update();
    }
}
```

## Advanced: Custom Sequences

```java
// Override AprilTag with custom sequence
BallColor[] customSequence = {
    BallColor.GREEN,
    BallColor.PURPLE,
    BallColor.GREEN
};

AutoShooter shooter = new AutoShooter(robot, ballSlots, customSequence);
Actions.runBlocking(shooter.shootAction());
```

## Troubleshooting

**Balls not firing in order:**
- Check that `BallSlot.getColor()` matches detected colors
- Verify AprilTag sequence is populated before creating shooter
- Use telemetry to confirm "Skipped" messages

**Shooter not spinning:**
- Check `RobotActionConfig.SHOOTER_POWER` (should be ~0.85)
- Verify `robot.shooterMotor` is initialized
- Check power distribution

**Gates not opening:**
- Verify `GATEUP` and `GATEDOWN` servo positions in `RobotActionConfig`
- Check servo wiring and direction

**Timing issues:**
- Adjust `SPIN_UP_TIME`, `FIRE_DELAY`, `GATE_OPEN_DELAY` constants
- Increase delays for heavier balls or slower shooter

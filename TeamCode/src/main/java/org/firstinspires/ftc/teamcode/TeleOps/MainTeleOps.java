package org.firstinspires.ftc.teamcode.TeleOps;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.DEBOUNCE_THRESHOLD;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Vision.AprilTagUpdate;

import java.util.Arrays;
import java.util.List;

@TeleOp(name = "TeleOps_Decode_gw", group = "org.firstinspires.ftc.teamcode")
public class MainTeleOps extends OpMode {

    //======================= Enums & Constants ===============================
    public enum ControlState {RUN, TEST}
    public enum BallHandlingState {IDLE, INTAKING, OFFTAKING}
    public enum IsAprilTagSequence {TRUE, FALSE}
    public enum AllianceSide {BLUE, RED}

    private final double[] spindexerSlotAngles = {
            RobotActionConfig.spindexerSlot1,
            RobotActionConfig.spindexerSlot2,
            RobotActionConfig.spindexerSlot3
    };

    //======================= Robot Components ================================
    private RobotHardware robot;
    private RobotDrive robotDrive;
    private IntakeBall intakeBall;
    private OffTakeBall offTakeBall;

    private ColorDetection colorDetection;
    private SlotList slotList;
    private AprilTagUpdate aprilTagUpdate;
    private ShooterDiscreteZonePowerTable powerTable;

    //======================= Controllers & State =============================
    private GamepadEx gamepadCo1, gamepadCo2;
    private ControlState controlState = ControlState.RUN;
    private BallHandlingState ballHandlingState = BallHandlingState.IDLE;
    private AllianceSide allianceSide = AllianceSide.RED;
    private IsAprilTagSequence isAprilTagSequence = IsAprilTagSequence.FALSE;
    private boolean useAprilTagSequence = false;
    private Pose2D targetGoalPos = new Pose2D(DistanceUnit.INCH,72, 72, AngleUnit.DEGREES,45);

    //======================= Timers & Helpers ================================
    private ElapsedTime debounceTimer = new ElapsedTime();
    private ElapsedTime runTime = new ElapsedTime();

    private ElapsedTime ledFlashTimer = new ElapsedTime();
    private boolean lBstartPressed = false;

    //======================= Overridden OpMode Methods =======================
    @Override
    public void init() {
        // Telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware and Subsystems
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initPinPoint();

        // Controllers
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);

        // Drivetrain
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();
        powerTable = new ShooterDiscreteZonePowerTable();

        // Ball Handling Subsystems
        slotList = new SlotList(spindexerSlotAngles);
        intakeBall = new IntakeBall(robot, gamepadCo2, slotList.getBalls(), spindexerSlotAngles);
        offTakeBall = new OffTakeBall(robot, gamepadCo2, slotList.getBalls(), powerTable);

        // Vision
        aprilTagUpdate = new AprilTagUpdate(hardwareMap);

        // Initial States
        initializeServos();
        initializeAprilTagDefaults();
        enableBulkCaching();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Control Mode", robotDrive.getDriveMode().name());
        telemetry.update();
    }

    @Override
    public void init_loop() {
        handleAprilTagDetection();
        handleAllianceSelection();
        telemetry.addData("Alliance Side", allianceSide.name());
    }

    @Override
    public void start() {
        debounceTimer.reset();
        runTime.reset();
        ledFlashTimer.reset();
    }

    @Override
    public void loop() {
        //--- Universal Updates ---
        // Always update odometry and calculate distance, regardless of mode
        robot.pinPoint.update();
        double distanceToGoal = getTargetGoalDist(targetGoalPos);
        offTakeBall.setDistanceToGoal(distanceToGoal);

        //--- handle led indicator ---
        handleLedIndicator(distanceToGoal);

        //--- Input Handling ---
        handleDriverInputs();

        //--- Main Logic ---
        if (controlState == ControlState.RUN) {
            runMainLogic();
        }

        //--- Telemetry ---
        displayTelemetry();
    }

    @Override
    public void stop() {
        robot.stopAllMotors();
        intakeBall.stopIntake();
        offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.OFFTAKE_IDLE);
        telemetry.addData("Status", "Robot Stopped");
        telemetry.update();
    }

    //======================= Main Logic Methods ==============================
    /**
     * Handles the primary robot functions during the RUN state.
     */
    private void runMainLogic() {
        robotDrive.DriveLoop();
        runBallHandlingFSM();
    }

    /**
     * Manages the state transitions for the ball handling system (Intake, Offtake, Idle).
     */
    private void runBallHandlingFSM() {
        switch (ballHandlingState) {
            case INTAKING:
                intakeBall.IntakeBallUpdate();
                intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
                offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.OFFTAKE_DONE);
                if (intakeBall.isFull()) {
                    ballHandlingState = BallHandlingState.IDLE;
                }
                break;
            case OFFTAKING:
                offTakeBall.update();
                intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
                if (offTakeBall.isSortingComplete()) {
                    ballHandlingState = BallHandlingState.IDLE;
                }
                break;
            case IDLE:
                intakeBall.stopIntake();
                intakeBall.setState(IntakeBall.INTAKEBALLSTATE.INTAKE_READY);
                offTakeBall.setState(OffTakeBall.OFFTAKEBALLSTATE.OFFTAKE_DONE);
                offTakeBall.update();
                break;
        }
    }

    //======================= Input Handling Methods ==========================
    /**
     * Checks for all controller inputs and triggers corresponding actions.
     */
    private void handleDriverInputs() {
        handleControlStateToggle();
        handleAprilTagSequenceToggle();
        handleAutomaticSequenceTrigger();
        handleBallHandlingInputs();
        handleOdomReset();
    }

    /**
     * Handles controller inputs for the main ball handling state machine.
     */
    private void handleBallHandlingInputs() {
        if (gamepadCo2.getButton(A) && isDebounced()) {
            ballHandlingState = BallHandlingState.INTAKING;
        }

        if (gamepadCo2.getButton(X) && isDebounced()) {
            ballHandlingState = BallHandlingState.OFFTAKING;
            intakeBall.stopIntake();
        }

        if (gamepadCo2.getButton(B) && isDebounced()) {
            ballHandlingState = BallHandlingState.IDLE;
        }
    }

    /**
     * Toggles between RUN and TEST modes.
     */
    private void handleControlStateToggle() {
        if (gamepadCo1.getButton(START) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed && isDebounced()) {
            lBstartPressed = true;
            controlState = (controlState == ControlState.RUN) ? ControlState.TEST : ControlState.RUN;
        } else if (!gamepadCo1.getButton(START) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }
    }

    /**
     * Toggles whether to use the detected AprilTag sequence for sorting.
     */
    private void handleAprilTagSequenceToggle() {
        if (gamepadCo2.getButton(GamepadKeys.Button.DPAD_UP) && isDebounced()) {
            isAprilTagSequence = (isAprilTagSequence == IsAprilTagSequence.TRUE) ? IsAprilTagSequence.FALSE : IsAprilTagSequence.TRUE;
            if (isAprilTagSequence == IsAprilTagSequence.TRUE) {
                offTakeBall.setSequence(SharedColorSequence.aprilTagSequence);
            } else {
                offTakeBall.setSequence(null); // Or a default sequence
            }
        }
    }
    /**
     * NEW: Resets the Pinpoint odometry to zero when the driver presses DPAD_DOWN.
     */
    private void handleOdomReset() {
        if (gamepadCo1.getButton(DPAD_DOWN) && isDebounced()) {
            robot.pinPoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0,AngleUnit.DEGREES,0));
            telemetry.addLine("!!! ODOMETRY RESET !!!");
        }
    }

    /**
     * Automatically loads the AprilTag sequence after 100 seconds.
     */
    private void handleAutomaticSequenceTrigger() {
        if (runTime.seconds() > 100.0 && !useAprilTagSequence) {
            useAprilTagSequence = true;
            offTakeBall.setSequence(SharedColorSequence.aprilTagSequence);
        }
    }
    //=======================Handle LED ==========================
    /// Handel led
    private void handleLedIndicator(double distanceToGoal) {
        // Condition to start flashing (e.g., close to the goal)
        if (distanceToGoal < 50) {
            // Check the timer to toggle the LED on and off
            // This creates a 1-second cycle (500ms on, 500ms off)
            if (ledFlashTimer.milliseconds() < 500) {
                robot.rgbLED.setPosition(0.277); // "On" state (e.g., Red)
            } else if (ledFlashTimer.milliseconds() < 1000) {
                robot.rgbLED.setPosition(0.9); // "Off" state (or a dim color)
            } else {
                ledFlashTimer.reset(); // Reset the timer to repeat the flash cycle
            }
        } else {
            // If not in flashing mode, use the solid color logic
            if (intakeBall.getDetectedColor() == BallColor.GREEN) {
                robot.rgbLED.setPosition(0.5); // Solid Green
            } else if (intakeBall.getDetectedColor() == BallColor.PURPLE) {
                robot.rgbLED.setPosition(0.722); // Solid Purple
            } else if (intakeBall.getDetectedColor() == BallColor.UNKNOWN){
                robot.rgbLED.setPosition(0.388); // Default solid color (e.g., Orange)
            } else if (intakeBall.isFull()) {
                robot.rgbLED.setPosition(1.0); // Solid White
            }
        }
    }

    //======================= Initialization Helpers ==========================
    private void initializeServos() {
        robot.spindexerServo.setPosition(spindexerSlotAngles[0]);
        robot.leftGateServo.setPosition(RobotActionConfig.GATEUP);
        robot.rightGateServo.setPosition(RobotActionConfig.GATEUP);
    }

    private void initializeAprilTagDefaults() {
        SharedColorSequence.aprilTagSequence = new BallColor[]{BallColor.UNKNOWN, BallColor.UNKNOWN, BallColor.UNKNOWN};
        offTakeBall.setSequence(SharedColorSequence.aprilTagSequence);
        telemetry.addLine("Default sequence loaded. Waiting for AprilTag...");
    }

    private void enableBulkCaching() {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    //======================= Init_Loop Helpers ===============================
    private void handleAprilTagDetection() {
        aprilTagUpdate.update();
        int detectedID = aprilTagUpdate.getTagID();
        BallColor[] tagSequence = aprilTagUpdate.getSequence();
        if (detectedID != -1 && tagSequence != null) {
            SharedColorSequence.aprilTagSequence = tagSequence;
            telemetry.addData("AprilTag ID Found", detectedID);
            telemetry.addData("Detected Sequence", aprilTagUpdate.getSequenceAsString());
        } else {
            telemetry.addLine("Searching for AprilTag...");
        }
    }

    private void handleAllianceSelection() {
        if (gamepadCo1.getButton(BACK) && gamepadCo1.getButton(LEFT_BUMPER) && !lBstartPressed && isDebounced()) {
            lBstartPressed = true;
            allianceSide = (allianceSide == AllianceSide.RED) ? AllianceSide.BLUE : AllianceSide.RED;
            // Update target goal position based on new alliance side
            if (allianceSide == AllianceSide.BLUE) {
                targetGoalPos = new Pose2D(DistanceUnit.INCH,-70, -70, AngleUnit.DEGREES,45);
            } else {
                targetGoalPos = new Pose2D(DistanceUnit.INCH,-70, 70, AngleUnit.DEGREES,-45);
            }
        } else if (!gamepadCo1.getButton(BACK) || !gamepadCo1.getButton(LEFT_BUMPER)) {
            lBstartPressed = false;
        }
    }

    //======================= General Utility Methods =========================
    private boolean isDebounced() {
        if (debounceTimer.milliseconds() > DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    private double getTargetGoalDist(Pose2D targetPos) {
        double deltaX = targetPos.getX(DistanceUnit.INCH) - robot.pinPoint.getPosX(DistanceUnit.INCH);
        double deltaY = targetPos.getY(DistanceUnit.INCH) - robot.pinPoint.getPosY(DistanceUnit.INCH);
        return Math.hypot(deltaX, deltaY);
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0 && voltage < result) {
                result = voltage;
            }
        }
        return (result == Double.POSITIVE_INFINITY) ? 0.0 : result;
    }

    //======================= Telemetry Display ===============================
    private void displayTelemetry() {
        telemetry.addLine("--- Op Mode ---");
        telemetry.addData("Run Time", "%.1f sec", runTime.seconds());
        telemetry.addData("Run Mode", controlState);
        telemetry.addData("Ball Handling", ballHandlingState.name());
        telemetry.addData("Drive Mode", robotDrive.getDriveMode().name());
        telemetry.addData("Battery Voltage", "%.2f V", getBatteryVoltage());

        telemetry.addLine("--- Odometry ---");
        telemetry.addData("X", "%.2f in", robot.pinPoint.getPosX(DistanceUnit.INCH));
        telemetry.addData("Y", "%.2f in", robot.pinPoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("Heading", "%.2f°", robot.pinPoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Distance to Goal", "%.2f in", getTargetGoalDist(targetGoalPos));

        telemetry.addLine("--- Intake ---");
        telemetry.addData("State", intakeBall.getState());
        telemetry.addData("Ball Count", intakeBall.getNumberOfBalls());
        telemetry.addData("Detected Color", intakeBall.getDetectedColor().name());


        telemetry.addLine("--- Offtake ---");
        telemetry.addData("State", offTakeBall.getState());
        telemetry.addData("Using Color Sequence", offTakeBall.isColorSequence());
        telemetry.addData("Shooter Power", "%.2f", offTakeBall.getShooterPower());

        telemetry.addLine("--- Shared Ball Slots ---");
        for (BallSlot b : slotList.getBalls()) {
            telemetry.addData(
                    "Slot " + b.getSlotPosition(),
                    "HasBall=%s | Color=%s",
                    b.hasBall() ? "YES" : "NO",
                    b.getColor().name());
        }

        telemetry.addLine("--- AprilTag ---");
        telemetry.addData("Tag ID Found", aprilTagUpdate.getTagID());
        telemetry.addData("Sequence Mode", useAprilTagSequence ? "TAG-GUIDED" : "MANUAL");
        telemetry.addData("Loaded Sequence", Arrays.toString(SharedColorSequence.aprilTagSequence));

        if (controlState == ControlState.TEST) {
            telemetry.addLine("----- TEST MODE ACTIVE -----");
        }
        telemetry.update();
    }
}


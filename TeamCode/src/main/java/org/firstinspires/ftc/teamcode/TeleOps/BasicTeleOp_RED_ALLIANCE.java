package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.FSMIntake.IntakeStates;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SHOOTERSTATE;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueAllianceResetPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerRetract;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redAllianceResetPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;

import java.util.ArrayList;
import java.util.List;

/**
 * Making the sequence shooting go slot 5-4-3 (intaking) 3-2-1 (shooting)
 * Put blue alliance code (Turret, LUT, ect)
 * Sort shooting
 * Adding Motif Detection to auto
 * Adding Pose2D storage + turret heading from auto to teleOp
 * Turret PIDF values for shooter - coach did
 ---------------------------------------------------------------------------
 * Change shooting timing values when hardware is changed for rapid shooting
 * Tuning LUT values when the hardware changes
 */
@Config
@TeleOp(name = "---------RED--🐧---Coach_version-------", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp_RED_ALLIANCE extends OpMode {
    private long waitTimeMs;

    /// Enum states for robot action state
    public enum RobotActionState {
        Sequence_Shooting,
        Intaking,
        Idle
    }
    /// Enum states for alliance
    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }
    /// robot and subsystem
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private GamepadComboInput gamepadComboInput;
    private RobotDrive robotDrive;
    private FSMShooter FSMShooter;
    private FSMIntake FSMIntake;
    private TurretUpd turret;
    private SpindexerManualControl spindexerManualControl;
    private SpindexerUpd spindexer;
    private Limelight limelight;
    /// ----------------------------------------------------------------
    // For shooter power and angle calculator
    private ShooterPowerCalculator shooterPowerAngleCalculator;

    /// Time and frequency
    private long lastLoopTimeNs;
    private double loopHz = 0.0;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private static double voltage;

    /// ----------------------------------------------------------------
    /// For robot action state
    public RobotActionState actionStates;
    private RobotActionState requestedActionState = RobotActionState.Idle;
    private RobotActionState activeActionState = RobotActionState.Idle;
    private RobotActionState pendingActionState = null;
    private boolean stopRequestedForPending = false;

    /// For alliance colour
    public static Alliance alliance;

    /// For dashboard & Telemetry
    public static double shooterRPM;
    public static int shooterTargetRPM;
    private boolean resetTurret = false;
    private int startingTick;
    private TeleOpTelemetryManager teleOpTelemetryManager;
    public static boolean SIMPLE_TELEMETRY = false;

    /// For expensive values from getter
    private int currentZone;
    private double currentDistance;
    private double currentTx;
    private Pose2D currentPose;
    private double batteryVoltage;


    //========================================
    //---------- Initialization --------------
    //========================================
    @Override
    public void init() {
        /// For telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        teleOpTelemetryManager = new TeleOpTelemetryManager(telemetry);
        teleOpTelemetryManager.setUpdateEveryLoops(5);

        /// Instantized Robot subsystems and Hardware
        robot = new RobotHardware(hardwareMap);
        robot.init();                           //Initialize all motors and servos
        robot.initIMU();                        //Initialize control hub IMU
        robot.initPinpoint();                   //Initialize pinpoint
        robot.initializeBulkReading(hardwareMap);

        /// * Transfer the pose 2D from Auto Ops *
        Pose2d endPose = PoseStorage.currentPose;
        double heading_Radiant = endPose.heading.toDouble();
        Pose2D startingPose = new Pose2D(DistanceUnit.MM, PoseStorage.currentPose.position.x*25.4, PoseStorage.currentPose.position.y*25.4, AngleUnit.DEGREES, Math.toDegrees(heading_Radiant));
        robot.pinpoint.setPosition(startingPose);

        /// 0. gamepad---------------------------------------------------------------
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        gamepadComboInput = new GamepadComboInput(gamepadCo1,gamepadCo2);

        /// 1. robot drive-------------------------------------------------------------
        robotDrive = new RobotDrive(robot, gamepadComboInput);
        robotDrive.Init();

        /// 2. color detection------------------------------------------------------------
        /// empty

        /// 3. limelight--------------------------------------------------------------
        limelight = new Limelight(robot);
        limelight.initLimelight(24);
        limelight.start();

        /// 4.spindexer-------------------------------------------------------------------
        spindexer = new SpindexerUpd(robot, SpindexerUpd.SLOT.Empty, SpindexerUpd.SLOT.Empty, SpindexerUpd.SLOT.Empty, 0);
        spindexerManualControl = new SpindexerManualControl(robot, spindexer, gamepadComboInput);

        ///  Alliance selection
        alliance =
                Alliance.RED_ALLIANCE;
        boolean isRedAlliance = alliance == Alliance.RED_ALLIANCE;

        /// 5. turret---------------------------------------------------------------
        turret = new TurretUpd(robot, isRedAlliance);

        /// 6 power calculator for shooter------------------------------------------------------------
        shooterPowerAngleCalculator = new ShooterPowerCalculator(robot);
        shooterPowerAngleCalculator.setAlliance(isRedAlliance);

        /// 7. shooter-------------------------------------------------------------
        FSMShooter = new FSMShooter(robot, spindexer, shooterPowerAngleCalculator, gamepadComboInput, turret, limelight);
        FSMShooter.Init();

        /// 8. intake------------------------------------------------------------
        FSMIntake = new FSMIntake (robot, spindexer);

        /// 9. alliance selection-----------------------------------------------------------
        alliance = Alliance.RED_ALLIANCE;
        if (alliance == Alliance.RED_ALLIANCE) {
            shooterPowerAngleCalculator.setAlliance(true);
        }else{
            shooterPowerAngleCalculator.setAlliance(false);
        };

        /// 10. robot state----------------------------------------------------------
        actionStates = RobotActionState.Idle;

        /// 11. start adjuster servo at position to avoid soft start
        robot.shooterAdjusterServo.setPosition(0.48);

        /// 12. start kicker servo at position to avoid soft start
        robot.kickerServo.setPosition(kickerRetract);

        ///  13. Telemetry------------------------------------------------------------
        telemetry.addData("start pose", PoseStorage.currentPose);
        telemetry.addData("pinpoint", robot.pinpoint.getPosition());
        telemetry.update();
    }

    /// ------Main Loop----------------------------------------------------------
    @Override
    public void loop() {
        // ========================================================
        // WORKING FLOW:
        // 1.updateActionStateTransitions() decides when safe to enter
        // 2.requestGracefulStopsIfNeeded(target) requests STOP sequence
        // 3.onEnterActionState(target) starts the new sequence only
        // 4.PER-ACTION FSM switch uses activeActionState and does not set FSM states
        //---------------------------------------------------------
        //OPERATION FLOW
        // 1.Buttons → requestedActionState
        // 2.Transition manager:-> requests graceful stops -> waits for canExit() ->commits activeActionState
        //                     -> calls onEnterActionState()
        // 3. Pre-loop switch ->only adds behavior, never changes FSM states
        // --------VISUAL MIND MODEL-------------------------------
        //      [Buttons]
        //      ↓
        //      requestedActionState
        //      ↓
        //      updateActionStateTransitions()
        //      ↓
        //      activeActionState
        //      ↓
        //      onEnterActionState()   (once)
        //      ↓
        //      ┌─────────────────────────────┐
        //      │  Pre-loop switch (extras)   │  ← turret / LEDs / telemetry only
        //      └─────────────────────────────┘
        //      ↓
        //      FSMIntake.loop()
        //      FSMShooter.loop()
        // =========================================================

        // =========================================================
        // 1. START A NEW HARDWARE LOOP
        // =========================================================

        /*
         * Clear all REV Hub bulk caches exactly once at the
         * beginning of the loop.
         */
        robot.clearBulkCache();

        // =========================================================
        // 2. GAMEPAD INPUT
        // =========================================================

        gamepadCo1.readButtons();
        gamepadCo2.readButtons();

        gamepadComboInput.update();

        /*
         * Updates requestedActionState only.
         */
        buttonUpdate();

        // =========================================================
        // 3. UPDATE ACTION TRANSITIONS
        // =========================================================

        /*
         * Run the transition manager before subsystem FSMs.
         *
         * If a subsystem becomes safe, the new action state can be
         * entered and processed during this same loop.
         */
        updateActionStateTransitions();

        // =========================================================
        // 4. SENSOR UPDATES
        // =========================================================

        /*
         * Update Pinpoint exactly once.
         *
         * TurretUpd and ShooterPowerCalculator will then read the
         * current Pinpoint values without calling update() again.
         */
        robot.pinpoint.update();

        updateLoopFrequency();

        // =========================================================
        // 5. DRIVE
        // =========================================================

        /*
         * Driving remains responsive regardless of the active
         * intake or shooter state.
         */
        robotDrive.DriveLoop();

        // =========================================================
        // 6. SUBSYSTEM FSMs
        // =========================================================

        /*
         * Always run both FSMs so their timers, stopping sequences
         * and state transitions can continue.
         */
        FSMIntake.loop();

        /*
         * SequenceShooterLoop performs:
         *
         * - Shooter state update
         * - Shooter PID + feedforward calculation
         * - Shooter motor control
         * - Goal-zone selection
         * - Turret sensor snapshot
         * - Limelight query
         * - Turret motion-profile PIDF
         * - Spindexer update
         */
        FSMShooter.SequenceShooterLoop();

        // =========================================================
        // 7. ACTION-SPECIFIC EXTRA CONTROLS
        // =========================================================

        switch (activeActionState) {

            case Sequence_Shooting:
                /*
                 * Shooter and turret are handled by FSMShooter.
                 */
                break;

            case Intaking:
                /*
                 * Intake is handled by FSMIntake.
                 */
                break;

            case Idle:
                /*
                 * Manual Spindexer control is available only while
                 * the overall action state is idle.
                 *
                 * Placing this after the FSM updates makes manual
                 * commands the final Spindexer command this loop.
                 */
                spindexerManualControl.loop();
                break;

            default:
                break;
        }

        // =========================================================
        // 8. COLLECT CACHED TELEMETRY VALUES
        // =========================================================

        /*
         * These getters now return values already calculated during
         * this loop. They should not perform new sensor updates.
         */
        currentPose =
                robot.pinpoint.getPosition();

        currentZone =
                shooterPowerAngleCalculator
                        .getCurrentZone();

        currentDistance =
                shooterPowerAngleCalculator
                        .getDistance();

        currentTx =
                FSMShooter.getLimelightTxForLED();

        batteryVoltage =
                robot.getBatteryVoltageRobust();

        shooterRPM =
                shooterPowerAngleCalculator
                        .getMeasureRPM();

        shooterTargetRPM =
                shooterPowerAngleCalculator
                        .getRPM();
        waitTimeMs = FSMShooter.getTimestampForGoalZone(currentZone);
        // =========================================================
        // 9. LED
        // =========================================================

        updateLED(currentTx);

        // =========================================================
        // 10. TELEMETRY
        // =========================================================
        teleOpTelemetryManager.updateSimplified(
                alliance,
                requestedActionState,
                activeActionState,
                FSMIntake,
                FSMShooter,
                spindexer,
                loopHz,
                currentZone,
                currentDistance,
                currentTx,
                batteryVoltage,
                shooterRPM,
                shooterTargetRPM,
                waitTimeMs
        );
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        robot.topShooterMotor.setPower(0.0);
        robot.bottomShooterMotor.setPower(0.0);

        robot.turretMotor.setPower(0.0);
        /*
         * Stop Limelight processing if your Limelight wrapper
         * provides this method.
         */
        limelight.stop();
    }

    // =========================================================
    // Update Action State based on button presses
    // - prevent change Action State when FSM not finish
    // =========================================================
    private void updateActionStateTransitions() {

        // If no transition in progress, see if we need to start one
        if (pendingActionState == null) {
            if (requestedActionState == activeActionState) return;

            pendingActionState = requestedActionState;   // LATCH target
            stopRequestedForPending = false; //
        }

        // initial intake_safe_stop and shooter_safe_stop boolean
        boolean intakeSafe  = FSMIntake.canExit();
        boolean shooterSafe = FSMShooter.canExit();

        // 2) OPTIONAL: allow retargeting only when shooter is safe
        if (pendingActionState != null && shooterSafe) {
            pendingActionState = requestedActionState;
        }
        // ------------------------------------------------------------
        // 1) If we are NOT safe yet, request graceful stop(s)
        //    (this will not hard-cut; it triggers each FSM's stop sequence)
        // ------------------------------------------------------------
        //TODO: if specific stop needed. uncomment the function below
        // Request graceful stops once
        if (!stopRequestedForPending) {
            requestGracefulStopsIfNeeded(pendingActionState);
            stopRequestedForPending = true;
        }

        // ------------------------------------------------------------
        // 2) Decide if we are allowed to switch into the requested state
        // ------------------------------------------------------------
        boolean canSwitch = false;

        switch (pendingActionState) {
            case Intaking:
                canSwitch = shooterSafe;
                break;
            case Sequence_Shooting:
                canSwitch = intakeSafe;  // wait until intake finishes its current sequence
                break;
            case Idle:
            default:
                // safest: wait for both to finish their sequences
                canSwitch = intakeSafe && shooterSafe;
                break;
        }
        // ------------------------------------------------------------
        // 3) Commit switch only when safe
        // ------------------------------------------------------------
        if (canSwitch) {
            // commit the switch
            activeActionState = pendingActionState;
            pendingActionState = null;          // clear transition
            stopRequestedForPending = false;
            onEnterActionState(activeActionState);
        }
    }
    // =========================================================
    // CHECK SAFE EXIT
    // ONLY SHOOTER STOPPED, IT CAN SAFE EXIT / SWITCH
    // NOT SAFE, REQUEST TO GRACEFULSTOP - FSMIntake.requestGracefulStop()
    //===========================================================
    private void requestGracefulStopsIfNeeded(RobotActionState target) {
        switch (target) {
            case Sequence_Shooting:
                if (!FSMIntake.canExit()) {
                    FSMIntake.requestGracefulStop(); // sets INTAKE_STOP if needed
                }
                break;

            case Intaking:
                // We are trying to intake → shooter must finish gracefully first
                break;

            case Idle:
            default:
                // Going idle → stop both gracefully
                if (!FSMIntake.canExit()) {
                    FSMIntake.requestGracefulStop();
                }
                break;
        }
    }
    //===========================================================
    // Enter Action State
    // Actually set FSM STATES
    //===========================================================
    private void onEnterActionState(RobotActionState s) {
        switch (s) {
            case Sequence_Shooting:
                FSMShooter.shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                break;

            case Intaking:
                FSMIntake.intakeStates  = IntakeStates.INTAKE_PREP;
                break;

            case Idle:
            default:
                // Don't hard-cut motors here unless your FSM is already idle-safe.
                // Better: request FSMs to go to IDLE naturally.
                // Graceful stop is handled by requestSubsystemStopForTransition()
                break;
        }
    }
    //===========================================================
    // button updates
    // Assign button for FSM states -
    //===========================================================

    public void buttonUpdate() {
        boolean idlePressed     = gamepadComboInput.getBPressedAny();
        boolean seqShootPressed = gamepadComboInput.getXPressedAny();

        boolean intakePressed   = gamepadComboInput.getDpadLeftPressedAny();

        boolean reversePressed  = gamepadComboInput.getDpadRightPressedAny(); // available if you add enum/action, right now it is the same as B idle pressed
        boolean dpDownPressed   = gamepadComboInput.getDpadDownPressedAny();

        boolean aPressed        = gamepadComboInput.getAPressedAny();         // available
        boolean yPressed        = gamepadComboInput.getYPressedAny();         // available

        // ==========================
        // Action State Priority
        // ==========================
        if (idlePressed) {
            requestedActionState = RobotActionState.Idle;

        } else if (seqShootPressed) {
            requestedActionState = RobotActionState.Sequence_Shooting;

        } else if (intakePressed) {
            requestedActionState = RobotActionState.Intaking;
        }

        // ==========================
        // Pose reset (DPAD_DOWN)
        // ==========================
        if (dpDownPressed) {
            robot.pinpoint.setPosition(
                    alliance == Alliance.RED_ALLIANCE ? redAllianceResetPose : blueAllianceResetPose
            );
        }

    }
    ///  helper functions
    ///  - LED Update
    private void updateLED(double tx) {
        if (Double.isNaN(tx)) {
            robot.LED.setPosition(0.288); // red (no tag)
        }
        else if (Math.abs(tx) < 1.0) {
            robot.LED.setPosition(0.5);   // green (in deadband)
        }
        else if (tx >= 5.0 && tx < 10.0) {
            robot.LED.setPosition(0.388); // yellow (your desired 5~20) <-- set to your purple value
        }
        else if (tx <= -5.0 && tx > -10.0) {
            robot.LED.setPosition(0.722);  // purple (your desired -20~-5) <-- set to your yellow value
        }
        else {
            robot.LED.setPosition(0.0);   // default black (outside these ranges)
        }
    }

    ///  - Frequency Updates
    private void updateLoopFrequency() {
        long nowNs =
                System.nanoTime();

        if (lastLoopTimeNs != 0L) {
            double deltaTimeSeconds =
                    (nowNs - lastLoopTimeNs)
                            / 1_000_000_000.0;

            if (deltaTimeSeconds > 0.0) {
                loopHz =
                        1.0 / deltaTimeSeconds;
            }
        }

        lastLoopTimeNs = nowNs;
    }
}

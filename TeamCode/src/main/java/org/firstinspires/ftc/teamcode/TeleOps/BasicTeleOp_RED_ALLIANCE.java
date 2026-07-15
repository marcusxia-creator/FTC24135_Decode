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
 * Turret PIDF
 * Tuning PID values for shooter - coach did
 ---------------------------------------------------------------------------
 * Change shooting timing values when hardware is changed for rapid shooting
 * Tuning LUT values when the hardware changes
 */
@Config
@TeleOp(name = "---------RED--🐧----------", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp_RED_ALLIANCE extends OpMode {
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
    private Turret turret;
    private SpindexerManualControl spindexerManualControl;
    private SpindexerUpd spindexer;
    private Limelight limelight;
    /// ----------------------------------------------------------------
    // For shooter power and angle calculator
    private ShooterPowerCalculator shooterPowerAngleCalculator;

    // for ball color and color detection
    private BallColor ballColor;
    private ColorDetection colorDetection;

    // for time and frequency
    private long lastLoopTime = 0;
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


    public List<String> switchTickLog = new ArrayList<>();

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

        /// 6. color detection------------------------------------------------------------
        colorDetection = new ColorDetection(robot);

        /// 9. limelight--------------------------------------------------------------
        limelight = new Limelight(robot);
        limelight.initLimelight(24);
        limelight.start();

        /// 2.spindexer-------------------------------------------------------------------
        spindexer = new SpindexerUpd(robot, SpindexerUpd.SLOT.Empty, SpindexerUpd.SLOT.Empty, SpindexerUpd.SLOT.Empty, 0);
        spindexerManualControl = new SpindexerManualControl(robot, spindexer, gamepadComboInput);

        /// 3. turret---------------------------------------------------------------
        turret = new Turret(robot, true);

        /// 4.1. power calculator for shooter------------------------------------------------------------
        shooterPowerAngleCalculator = new ShooterPowerCalculator(robot);
        /// 4. shooter-------------------------------------------------------------
        FSMShooter = new FSMShooter(robot, spindexer, shooterPowerAngleCalculator, gamepadComboInput, turret, limelight);
        FSMShooter.Init();

        /// 5. intake------------------------------------------------------------
        FSMIntake = new FSMIntake (robot, spindexer);

        /// 7. alliance selection-----------------------------------------------------------
        alliance = Alliance.RED_ALLIANCE;
        if (alliance == Alliance.RED_ALLIANCE) {
            shooterPowerAngleCalculator.setAlliance(true);
        }else{
            shooterPowerAngleCalculator.setAlliance(false);
        };

        /// 8. robot state----------------------------------------------------------
        actionStates = RobotActionState.Idle;

        /// 10. start adjuster servo at position to avoid soft start
        robot.shooterAdjusterServo.setPosition(0.48);

        /// 11. start kicker servo at position to avoid soft start
        robot.kickerServo.setPosition(kickerRetract);

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
        // 1. INPUT UPDATE (read buttons + combos)
        // =========================================================
        gamepadCo1.readButtons();
        gamepadCo2.readButtons();
        /// combo button LB+ & RB+ config and update
        gamepadComboInput.update(); // for combined button combo
        /// Changes the action state base on which button is pressed
        buttonUpdate(); // sets requestedActionState ONLY

        // =========================================================
        // 2. CONTINUOUS SENSOR / HOUSEKEEPING UPDATES
        // =========================================================
        robot.pinpoint.update();
        ballColor = BallColor.fromHue(colorDetection.getHue());
        updateLoopFrequency();
        currentPose = robot.pinpoint.getPosition();
        currentZone = shooterPowerAngleCalculator.getCurrentZone();
        currentDistance = shooterPowerAngleCalculator.getDistance();
        currentTx = limelight.getTargetXForTag(24);
        batteryVoltage = robot.getBatteryVoltageRobust();

        // =========================================================
        // 3. DRIVE (always responsive)
        // =========================================================
        robotDrive.DriveLoop();

        // =========================================================
        // FIXME: 5.
        //  4. MODIFIED - PER-ACTION "EXTRAS" (NO FSM STATE FORCING HERE)
        // =========================================================
        switch (activeActionState){
            case Sequence_Shooting:
                //turret.driveTurretMotor();
                break;
            case Intaking:
                // empty as the FSM handles this,intake FSM already running
                break;
            case Idle:
                // empty as the FSM handles this
                spindexerManualControl.loop();
                break;
            default:
                // do nothing — graceful stop handled elsewhere
                break;
        }

        // =========================================================
        // 5. ZONE STATUS
        // =========================================================
        turret.updateZoneForGoalPose(currentZone);
        FSMShooter.updateZoneForGoalPose(currentZone);

        //Turret PIDF Config
        //turret.updatePidFromDashboard();

        // =========================================================
        // 6. SUBSYSTEM FSMs (ALWAYS RUN)
        // =========================================================
        FSMIntake.loop();
        // =========================================================
        // 6.1 SHOOTER & TURRET CONTROL (BUTTON CONTROLLED)
        // =========================================================
        /// When gamepad back pressed, reset turret.
        /// Otherwise, normal shooter and turret drive
        if (gamepadComboInput.getBackSinglePressedAny()) {
            resetTurret = true;
            startingTick = robot.turretMotor.getCurrentPosition(); // latch once on press
        }
        if (resetTurret) {
            if (turret.turretReset(startingTick)) {
                resetTurret = false;
                FSMShooter.resetTrim();
            }
        } else {
            //Limelight.TxSnapshot snap = limelight.getTxForTag(24);
            //FSMShooter.setLimelightTx(snap.hasTarget, snap.txDeg);
            FSMShooter.SequenceShooterLoop();
        }

        // =========================================================
        // 7. NEW! - ACTION STATE TRANSITION MANAGER (GRACEFUL)
        // Refine the order to reduce one loop delay.
        // =========================================================
        updateActionStateTransitions();

        // =========================================================
        // 8. LED STATUS (non-blocking)
        // =========================================================
        updateLED(currentTx);

        // =========================================================
        // 9. TELEMETRY
        // =========================================================
        if (SIMPLE_TELEMETRY) {
            teleOpTelemetryManager.updateSimplified(
                    alliance,
                    requestedActionState,
                    activeActionState,
                    FSMIntake,
                    FSMShooter,
                    spindexer,
                    robot,
                    shooterPowerAngleCalculator,
                    turret,
                    limelight,
                    loopHz
            );
        } else {
            teleOpTelemetryManager.update(
                    alliance,
                    requestedActionState,
                    activeActionState,
                    switchTickLog,
                    colorDetection,
                    FSMIntake,
                    FSMShooter,
                    spindexer,
                    robot,
                    shooterPowerAngleCalculator,
                    turret,
                    limelight,
                    loopHz
            );
        };
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
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

        // limit switch logging
        if (FSMShooter.turret.isLimitPressed()) {
            switchTickLog.add(Integer.toString(robot.turretMotor.getCurrentPosition()));
            if (switchTickLog.size() >= 10) {
                switchTickLog.remove(0);
            }
        }
    }

    ///  - Frequency Updates
    private void updateLoopFrequency() {
        long now = System.currentTimeMillis();
        if (lastLoopTime != 0) {
            long dtMs = now - lastLoopTime;
            if (dtMs > 0) {
                loopHz = 1000.0 / dtMs;
            }
        }
        lastLoopTime = now;
    }
}

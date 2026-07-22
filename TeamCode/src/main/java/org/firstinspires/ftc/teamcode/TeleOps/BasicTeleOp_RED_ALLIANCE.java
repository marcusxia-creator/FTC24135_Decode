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
        Shooting,
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
    FSMShooter FSMShooter;
    FSMIntake FSMIntake;
    private Turret turret;
    public Limelight limelight;
    /// ----------------------------------------------------------------
    // For shooter power and angle calculator
    private LUTPowerCalculator shooterPowerAngleCalculator;

    // for time and frequency
    private long lastLoopTime = 0;
    private double loopHz = 0.0;
    private double loopTime = 0.0;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private static double voltage;

    /// ----------------------------------------------------------------
    /// For robot action state
    public RobotActionState actionStates;

    private RobotActionState requestedActionState = RobotActionState.Idle;
    private RobotActionState activeActionState = RobotActionState.Idle;

    /// For alliance colour
    public static Alliance alliance;

    ///efficient telemetry
    public static double telemetryInterval;
    public ElapsedTime telemeteryTimer=new ElapsedTime();

    /// For expensive values from getter
    private int currentZone;
    private double currentDistance;
    private double currentTx;
    private double battertVoltage;
    private Pose2D cachedPosition;
    private double shooterMeasuredRPM;
    private int shooterTargetRPM;
    private double turretDriveAngle;
    private int turretCurrentTick;
    private int turretTargetTick;
    private double turretTargetAngle;
    private double turretMotorAngle;


    /// ----------------------------------------------------------------
    @Override
    public void init() {
        /// For telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryInterval = RobotActionConfig.telemetryInterval;
        /// For robot hardware initialization
        robot = new RobotHardware(hardwareMap);
        robot.init();                       //Initialize all motors and servos
        robot.initIMU();                    //Initialize control hub IMU
        robot.initPinpoint();               //Initialize pinpoint
        //robot.initExternalIMU();            //Initialize external IMU
        robot.initializeBulkReading(hardwareMap); //Initialize Bulk Reading

        /**
         * Transfer the pose 2D from Auto Ops
         */
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

        limelight = new Limelight(robot);
        limelight.initLimelight(25);
        limelight.start();

        /// 3. turret---------------------------------------------------------------
        turret = new Turret(robot, true);

        /// 4.1. power calculator for shooter------------------------------------------------------------
        shooterPowerAngleCalculator = new LUTPowerCalculator(robot);

        /// 4. shooter-------------------------------------------------------------
        FSMShooter = new FSMShooter(robot, shooterPowerAngleCalculator, gamepadComboInput, turret, limelight);
        FSMShooter.Init();

        /// 5. intake------------------------------------------------------------
        FSMIntake = new FSMIntake(robot);

        /// 7. alliance selection-----------------------------------------------------------
        alliance = Alliance.RED_ALLIANCE;
        shooterPowerAngleCalculator.setAlliance(true);

        /// 8. robot state----------------------------------------------------------
        actionStates = RobotActionState.Idle;

        /// 10. start adjuster servo at position to avoid soft start
        robot.shooterAdjusterServo.setPosition(0.48);

        robot.kickerServo.setPosition(kickerRetract);

        

    }

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
        // 3. Pre-loop switch ->only adds behaviour, never changes FSM states
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
        // 2. INPUT UPDATE (read buttons + combos)
        // =========================================================
        gamepadCo1.readButtons();
        gamepadCo2.readButtons();

        /// combo button LB+ & RB+ config and update
        gamepadComboInput.update(); // for combined button combo
        /// Changes the action state base on which button is pressed
        buttonUpdate(); // sets requestedActionState ONLY

        // =========================================================
        // 3. CONTINUOUS SENSOR / HOUSEKEEPING UPDATES
        // =========================================================
        robot.pinpoint.update();
        updateLoopFrequency();


        // =========================================================
        // 4. DRIVE (always responsive)
        // =========================================================
        robotDrive.DriveLoop();

        // =========================================================
        // 5. NEW! - ACTION STATE TRANSITION MANAGER (GRACEFUL)
        // =========================================================
        updateActionStateTransitions();

        // =========================================================
        // 6.. MODIFIED - PER-ACTION "EXTRAS" (NO FSM STATE FORCING HERE)
        // =========================================================
        switch (activeActionState){
            case Shooting:
                //turret.driveTurretMotor();
                break;
            case Intaking:
                // empty as the FSM handles this,intake FSM already running
                break;
            case Idle:
                // empty as the FSM handles this
                break;
            default:
                // do nothing — graceful stop handled elsewhere
                break;
        }

        // =========================================================
        // 7. ZONE STATUS
        // =========================================================
        int zone = shooterPowerAngleCalculator.getZone();
        turret.updateZoneForGoalPose(zone);
        FSMShooter.updateZoneForGoalPose(zone);

        //Turret PIDF Config
        turret.updatePidFromDashboard();

        // =========================================================
        // 8. SUBSYSTEM FSMs (ALWAYS RUN)
        // =========================================================
        FSMIntake.loop();
        FSMShooter.SequenceShooterLoop();

        // =========================================================
        // 9. LED STATUS (non-blocking)
        // =========================================================
        // read each hardware/derived value once per loop and share it
        // across LED status + telemetry instead of re-querying per use
        currentZone = shooterPowerAngleCalculator.getZone();
        currentTx = FSMShooter.getLimelightTxForLED();
        updateLED(currentTx);

        // =========================================================
        // 9. TELEMETRY
        // =========================================================
        cachedPosition = robot.pinpoint.getPosition();
        currentDistance = shooterPowerAngleCalculator.getDistance();
        //battertVoltage = robot.getBatteryVoltageRobust();
        shooterMeasuredRPM = shooterPowerAngleCalculator.getMeasureRPM();
        shooterTargetRPM = shooterPowerAngleCalculator.getRPMTarget();

        turretCurrentTick = turret.getCurrentTick();
        turretTargetTick = turret.getTargetTick();
        turretDriveAngle = turret.getTurretDriveAngle();
        turretTargetAngle = turret.getTargetAngle();
        turretMotorAngle = turret.getTurretMotorAngle();

        // runTimeTelemetry() calls telemetry.update() itself once telemetryInterval
        // elapses — don't call it again here or telemetry.update() (which pushes to
        // both Driver Station and FTC Dashboard) runs unthrottled every loop.
        /**
        runTimeTelemetry(
                cachedPosition, currentZone, turretCurrentTick, turretTargetTick
        );
         */
        debugTelemetry();
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

        if (requestedActionState == activeActionState) return;

        // Decide which subsystems must be "safe" before switching
        boolean intakeSafe  = FSMIntake.canExit();
        boolean shooterSafe = FSMShooter.canExit();

        // Example policy:
        // - Switching INTO Intaking requires shooter safe (so you don't intake while shooting)
        // - Switching INTO Shooting requires intake safe (so you don't stop intake mid-park)
        // - Switching INTO Idle requires both safe, or you can force a "stop request" first (recommended)
        // ------------------------------------------------------------
        // 1) If we are NOT safe yet, request graceful stop(s)
        //    (this will not hard-cut; it triggers each FSM's stop sequence)
        // ------------------------------------------------------------

        //TODO: if specific stop needed. uncomment the function below
        // requestGracefulstopIfNeeded - only for switching from intaking to shooting
        requestGracefulStopsIfNeeded(activeActionState, requestedActionState);

        // ------------------------------------------------------------
        // 2) Decide if we are allowed to switch into the requested state
        // ------------------------------------------------------------
        boolean canSwitch = false;

        switch (requestedActionState) {
            case Intaking:
                canSwitch = shooterSafe;
                break;
            case Shooting:
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
            activeActionState = requestedActionState;
            onEnterActionState(activeActionState);
        }
    }
    // =========================================================
    // TODO: not use, but if needed, Greacefulstop() stop the fms before naturally finish
    // CHECK SAFE EXIT
    // NOT SAFE, REQUEST TO GRACEFULSTOP - FSMShooter.requestGracefulStop()
    // NOT SAFE, REQUEST TO GRACEFULSTOP - FSMIntake.requestGracefulStop()
    //===========================================================
    private void requestGracefulStopsIfNeeded(RobotActionState current, RobotActionState target) {
        switch (target) {
            case Shooting:
                if (!FSMIntake.canExit()) {
                    FSMIntake.requestGracefulStop(); // sets INTAKE_STOP if needed
                }
                break;

            case Intaking:
                // Assume that shooter always runs all the way through
                break;

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
            case Shooting:
                FSMShooter.shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                break;

            case Intaking:
                FSMIntake.intakeStates  = IntakeStates.INTAKE_PREP;
                break;

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

    private void buttonUpdate() {
        boolean ShootPressed =
                (gamepadComboInput.getXPressedAny());
        boolean intakePressed =
                (gamepadComboInput.getDpadLeftPressedAny());
        boolean reversePressed =
                (gamepadComboInput.getDpadRightPressedAny());
        boolean idlePressed =
                (gamepadComboInput.getBPressedAny());
        boolean dpDown =
                (gamepadComboInput.getDpadDownPressedAny());
        boolean turretReset =
                (gamepadComboInput.getDriverBackSinglePressed());

        boolean sortPressed = gamepadComboInput.getOperatorLbXComboPressed(); // combo - LB+X for sorted shooting. Assume this is edge-based already
        if (ShootPressed) requestedActionState = RobotActionState.Shooting;
        if (intakePressed)   requestedActionState = RobotActionState.Intaking;
        //if (reversePressed)  requestedActionState = RobotActionState.Reverse_Intake; // add enum if needed
        if (idlePressed)     requestedActionState = RobotActionState.Idle;

        // Dpad down pose reset stays immediate (that's fine)
        if (dpDown) {
            if (alliance == Alliance.RED_ALLIANCE) {
                robot.pinpoint.setPosition(redAllianceResetPose);
            }
            if (alliance == Alliance.BLUE_ALLIANCE) {
                robot.pinpoint.setPosition(blueAllianceResetPose);
            }
        }
    }

    ///  helper functions
    /// - button debounce
    public boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
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

        long now = System.currentTimeMillis();

        if (lastLoopTime != 0) {
            long dtMs = now - lastLoopTime;
            loopTime=(double)dtMs/1000;
            if (dtMs > 0) {
                loopHz = 1000.0 / dtMs;
            }
        }

        lastLoopTime = now;
    }

    public void runTimeTelemetry(Pose2D cachedPosition, int zone, int turretCurrentTick, int turretTargetTick){
        //simplified telemetry for teleop
        if(telemeteryTimer.time()>telemetryInterval){
            telemetry.addData("loop frequency (Hz)", loopHz);
            telemetry.addData("Action State", actionStates);

            telemetry.addLine("\n---ROBOT");
            telemetry.addData("Alliance", alliance);
            telemetry.addData(
                    "Pose2D",
                    "X: %.2f, Y: %.2f, Heading: %.2f°",
                    cachedPosition.getX(DistanceUnit.INCH),
                    cachedPosition.getY(DistanceUnit.INCH),
                    cachedPosition.getHeading(AngleUnit.DEGREES)
            );
            telemetry.addData("Dist_to_Goal","%,.0f",currentDistance);

            telemetry.addLine("\n---INTAKE");
            telemetry.addData("Intake State", FSMIntake.intakeStates);

            telemetry.addLine("\n---SHOOTER");
            telemetry.addData("ShooterState", FSMShooter.shooterState);
            telemetry.addData("Shooter Zone", zone);
            telemetry.addData("Shooter Target RPM",shooterTargetRPM);
            telemetry.addData("Shooter actual RPM", shooterMeasuredRPM);

            telemetry.addLine("\n---TURRET");
            telemetry.addData("Turret state", FSMShooter.turretState);
            telemetry.addData("Turret trim", FSMShooter.trim);
            telemetry.addData("Turret error", turretCurrentTick-turretTargetTick);

            telemetry.addLine("\n---SPINDEXER");
            telemetry.addData("SD Current Pos", robot.spindexerServo.getPosition());

            telemetry.update();
            telemeteryTimer.reset();
        }
    }

    //===========================================================
    // telemetry Manager
    //===========================================================
    public void debugTelemetry(){
        // Full debug dump — gate it the same way as runTimeTelemetry() so switching
        // into debug mode can't push telemetry.update() every loop tick.
        if (telemeteryTimer.time() <= telemetryInterval) return;
        telemeteryTimer.reset();

        Pose2D pose = robot.pinpoint.getPosition(); // single pinpoint read — was read 3x below (getHeading + getPosition x2)
        double headingDeg = Math.toDegrees(pose.getHeading(AngleUnit.RADIANS));

        telemetry.addData("loop frequency (Hz)", loopHz);
        telemetry.addLine("-----");
        telemetry.addData("Action State", actionStates);
        telemetry.addData("Requested", requestedActionState);
        telemetry.addData("Active", activeActionState);
        telemetry.addData("IntakeState", FSMIntake.intakeStates);
        telemetry.addData("ShooterState", FSMShooter.shooterState);
        telemetry.addData("IntakeSafe", FSMIntake.canExit());
        telemetry.addData("ShooterSafe", FSMShooter.canExit());
        telemetry.addLine("-----");
        telemetry.addData("Slot 0", robot.slotSensor1.checkBall());
        telemetry.addData("Slot 1", robot.slotSensor2.checkBall());
        telemetry.addData("Slot 2", robot.slotSensor3.checkBall());
        telemetry.addData("Current Pos", robot.spindexerServo.getPosition());
        telemetry.addLine("-----");
        // cached value from this loop's SequenceShooterLoop() — shooterPowerAngleCalculator.getPower()
        // re-runs the shooter PID controller as a side effect, so don't call it just to display it
        telemetry.addData("shooter power calculator", FSMShooter.getPower());
        telemetry.addData("Shooter Power", robot.topShooterMotor.getPower());
        telemetry.addData("voltage from Shooter", FSMShooter.getVoltage());
        telemetry.addData("Shooter Target RPM",shooterTargetRPM);
        telemetry.addData("Shooter actual RPM","%,.0f",shooterMeasuredRPM);
        telemetry.addLine("-----");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("current angle", headingDeg);
        telemetry.addData("Pose2D", "X: %.2f  Y: %.2f  H: %.1f°",
                pose.getX(DistanceUnit.MM),pose.getY(DistanceUnit.MM),pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Starting Pose",PoseStorage.currentPose);
        telemetry.addData(
                "Pose (in)",
                "X: %.2f  Y: %.2f  H: %.1f°",
                pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), headingDeg
        );
        telemetry.addData("distance to goal", "%,.0f",currentDistance);
        telemetry.addData("Shooter Zone", currentZone);
        telemetry.addLine("Turret-----------------------------------");
        telemetry.addData("turret target angle", turretTargetAngle);
        telemetry.addData("turret drive angle", turretDriveAngle);
        telemetry.addData("turret motor angle", turretMotorAngle);
        //telemetry.addData("motor PIDF coefficient", robot.turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("current motor tick", turretCurrentTick);
        telemetry.addData("target motor tick", turretTargetTick);
        telemetry.addData("goal pose", turret.getGoalPose());
        telemetry.addLine("-----------------------------------------");
        ///telemetry.addData("limelight output", "%,.1f",limelight.normalizedPose2D(DistanceUnit.INCH));
        telemetry.addData("limelight angle Tx", currentTx);
        telemetry.update();
    }

}

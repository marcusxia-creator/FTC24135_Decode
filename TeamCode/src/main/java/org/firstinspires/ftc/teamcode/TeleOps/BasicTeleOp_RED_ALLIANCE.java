package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.FSMIntake.IntakeStates;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SHOOTERSTATE;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.INTAKE_TICKS_PER_REV;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.SHOOTER_RPM_CONVERSION;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueAllianceResetPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerRetract;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redAllianceResetPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Auto.Runs.commonclasses.PoseStorage;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;

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
@TeleOp(name = "RED_TELEOP_MEET_2", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp_RED_ALLIANCE extends OpMode {
    /// Enum states for robot action state
    public enum RobotActionState {
        Sequence_Shooting,
        Sort_Shooting,
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
    private GamepadInput gamepadInput;
    private RobotDrive robotDrive;
    FSMShooter FSMShooter;
    FSMIntake FSMIntake;
    private Turret turret;
    private SpindexerManualControl spindexerManualControl;
    //private Spindexer spindexer;
    private SpindexerSimp spindexer;
    public Limelight limelight;
    /// ----------------------------------------------------------------
    // For shooter power and angle calculator
    private LUTPowerCalculator shooterPowerAngleCalculator;

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

    /// For alliance colour
    public static Alliance alliance;

    /// for dashboard
    public static double shooterRPM;
    public static int shooterTargetRPM;

    /// ----------------------------------------------------------------
    @Override
    public void init() {
        /// For telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /// For robot hardware initialization
        robot = new RobotHardware(hardwareMap);
        robot.init();                       //Initialize all motors and servos
        robot.initIMU();                    //Initialize control hub IMU
        robot.initPinpoint();               //Initialize pinpoint
        robot.initExternalIMU();            //Initialize external IMU

        ///Pose2D startingPose = new Pose2D(DistanceUnit.INCH, PoseStorage.currentPose.position.x, PoseStorage.currentPose.position.y, AngleUnit.RADIANS, PoseStorage.currentPose.heading.real);
        ///robot.pinpoint.setPosition(startingPose);

        /// 0. gamepad---------------------------------------------------------------
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        gamepadInput = new GamepadInput(gamepadCo1,gamepadCo2);

        /// 1. robot drive-------------------------------------------------------------
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        /// 2.spindexer-------------------------------------------------------------------
        spindexer = new SpindexerSimp(robot, SpindexerSimp.SLOT.Empty, SpindexerSimp.SLOT.Empty, SpindexerSimp.SLOT.Empty, 0); //Change inits for comp

        // spindexer = new Spindexer(robot, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, 0); //Change inits for comp
        // spindexerManualControl = new SpindexerManualControl(robot, spindexer, gamepadInput);

        /// 3. turret---------------------------------------------------------------
        turret = new Turret(robot);

        /// 4.1. power calculator for shooter------------------------------------------------------------
        shooterPowerAngleCalculator = new LUTPowerCalculator(robot);

        /// 4. shooter-------------------------------------------------------------
        FSMShooter = new FSMShooter(gamepadCo1, gamepadCo2, robot, spindexer, shooterPowerAngleCalculator,gamepadInput);
        FSMShooter.Init();

        /// 5. intake------------------------------------------------------------
        FSMIntake = new FSMIntake(gamepadCo1, gamepadCo2, robot, spindexer);

        /// 6. color detection------------------------------------------------------------
        colorDetection = new ColorDetection(robot);

        /// 7. alliance selection-----------------------------------------------------------
        alliance = Alliance.RED_ALLIANCE;
        shooterPowerAngleCalculator.setAlliance(true);

        /// 8. robot state----------------------------------------------------------
        actionStates = RobotActionState.Idle;

        /// 9. limelight--------------------------------------------------------------
        limelight = new Limelight(robot, turret);
        limelight.initLimelight(24);
        limelight.start();



        /// 10. start adjuster servo at position to avoid soft start
        robot.shooterAdjusterServo.setPosition(0.49);

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
        gamepadInput.update(); // for combined button combo
        /// Changes the action state base on which button is pressed
        buttonUpdate(); // sets requestedActionState ONLY

        // =========================================================
        // 2. CONTINUOUS SENSOR / HOUSEKEEPING UPDATES
        // =========================================================
        robot.pinpoint.update();
        ballColor = BallColor.fromHue(colorDetection.getHue());
        updateLoopFrequency();


        // =========================================================
        // 3. DRIVE (always responsive)
        // =========================================================
        robotDrive.DriveLoop();

        //turret.driveTurretMotor();

        // =========================================================
        // FIXME: 4. NEW! - ACTION STATE TRANSITION MANAGER (GRACEFUL)
        // =========================================================
        updateActionStateTransitions();

        // =========================================================
        // FIXME: 5. MODIFIED - PER-ACTION "EXTRAS" (NO FSM STATE FORCING HERE)
        // =========================================================
        switch (activeActionState){
            case Sequence_Shooting:
                turret.driveTurretMotor();
                break;
            case Sort_Shooting:
                turret.driveTurretMotor();
                //FSMShooter.SortShooterLoop();
                break;
            case Intaking:
                // empty as the FSM handles this,intake FSM already running
                break;
            case Idle:
                // empty as the FSM handles this
            default:
                // do nothing — graceful stop handled elsewhere
                break;
        }

        // =========================================================
        // 6. SUBSYSTEM FSMs (ALWAYS RUN)
        // =========================================================
        FSMIntake.loop();
        FSMShooter.SequenceShooterLoop();

        // =========================================================
        // 7. LED STATUS (non-blocking)
        // =========================================================
        updateLED();

        // =========================================================
        // 8. TELEMETRY
        // =========================================================
        telemetryManager();
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

            case Sequence_Shooting:
                canSwitch = intakeSafe;  // wait until intake finishes its current sequence
                break;
            case Sort_Shooting:
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
            case Sequence_Shooting:
                if (!FSMIntake.canExit()) {
                    FSMIntake.requestGracefulStop(); // sets INTAKE_STOP if needed
                }
                break;

            case Sort_Shooting:
                // We are trying to start shooting → intake must finish gracefully first
                if (!FSMIntake.canExit()) {
                    FSMIntake.requestGracefulStop(); // sets INTAKE_STOP if needed
                }
                break;

            case Intaking:
                // We are trying to intake → shooter must finish gracefully first
                if (!FSMShooter.canExit()) {
                    //FSMShooter.requestGracefulStop(); // you implement: STOPPING or set IDLE safely
                }
                break;

            case Idle:
            default:
                // Going idle → stop both gracefully
                if (!FSMIntake.canExit()) {
                    FSMIntake.requestGracefulStop();
                }
                if (!FSMShooter.canExit()) {
                    //FSMShooter.requestGracefulStop();
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

            case Sort_Shooting:
                FSMIntake.intakeStates  = IntakeStates.INTAKE_STOP; // or IDLE
                // FSMShooter.sortShooterState = ...
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

    private void buttonUpdate() {
        boolean seqShootPressed =
                (gamepadCo1.getButton(GamepadKeys.Button.X) || gamepadCo2.getButton(GamepadKeys.Button.X))
                        && isButtonDebounced();
        boolean intakePressed =
                (gamepadCo1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_LEFT))
                        && isButtonDebounced();
        boolean reversePressed =
                (gamepadCo1.getButton(GamepadKeys.Button.DPAD_RIGHT) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_RIGHT))
                        && isButtonDebounced();
        boolean idlePressed =
                (gamepadCo1.getButton(GamepadKeys.Button.B) || gamepadCo2.getButton(GamepadKeys.Button.B))
                        && isButtonDebounced();

        boolean sortPressed = gamepadInput.getOperatorLbXComboPressed(); // combo - LB+X for sorted shooting. Assume this is edge-based already
        if (seqShootPressed) requestedActionState = RobotActionState.Sequence_Shooting;
        if (sortPressed)     requestedActionState = RobotActionState.Sort_Shooting;
        if (intakePressed)   requestedActionState = RobotActionState.Intaking;
        //if (reversePressed)  requestedActionState = RobotActionState.Reverse_Intake; // add enum if needed
        if (idlePressed)     requestedActionState = RobotActionState.Idle;

        // Dpad down pose reset stays immediate (that's fine)
        if (gamepadCo1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
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
    private void updateLED() {
        if (shooterPowerAngleCalculator.getZone() ==0) {
            //Distance outside shooting zone, red alert
            robot.LED.setPosition(0.28);
        }
        else if (shooterPowerAngleCalculator.getZone() > 0 && limelight.llresult()){
            robot.LED.setPosition(1.0);
        }
        else if (shooterPowerAngleCalculator.getZone() > 0 && !limelight.llresult()){
            robot.LED.setPosition(0.388);
        }
        else { //Default black
            robot.LED.setPosition(0.0);
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

    //===========================================================
    // telemetry Manager
    //===========================================================
    public void telemetryManager(){
        telemetry.addData("loop frequency (Hz)", loopHz);
        telemetry.addData("voltage from robot", robot.getBatteryVoltageRobust());
        telemetry.addLine("-----");
        telemetry.addData("Action State", actionStates);
        telemetry.addData("Requested", requestedActionState);
        telemetry.addData("Active", activeActionState);
        telemetry.addData("IntakeState", FSMIntake.intakeStates);
        telemetry.addData("ShooterState", FSMShooter.shooterState);
        telemetry.addData("IntakeSafe", FSMIntake.canExit());
        telemetry.addData("ShooterSafe", FSMShooter.canExit());
        telemetry.addLine("-----");
        telemetry.addData("Distance Sensor", robot.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Sensor Color", colorDetection.getStableColor());
        telemetry.addData("Sensor values", spindexer.colorValue);
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Pos", spindexer.currentPos);
        telemetry.addData("Shooter Target Colour", FSMShooter.targetColour.name());
        telemetry.addLine("-----");
        telemetry.addData("shooter power calculator", shooterPowerAngleCalculator.getPower());
        telemetry.addData("Shooter Power", robot.topShooterMotor.getPower());
        telemetry.addData("voltage from Shooter", FSMShooter.getVoltage());
        telemetry.addData("power set point", FSMShooter.getPower_setpoint());
        shooterTargetRPM = shooterPowerAngleCalculator.getRPM();
        shooterRPM = shooterPowerAngleCalculator.getMeasureRPM();
        telemetry.addData("Shooter Target RPM",shooterTargetRPM);
        telemetry.addData("Shooter acutal RPM",shooterRPM);
        telemetry.addData("Shooter RPM","%,.0f",robot.topShooterMotor.getVelocity()*SHOOTER_RPM_CONVERSION);
        telemetry.addLine("-----");
        String MotifAvailable;
        telemetry.addData("current angle", robot.pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose2D", robot.pinpoint.getPosition());
        telemetry.addData("distance to goal", "%,.0f",shooterPowerAngleCalculator.getDistance());
        telemetry.addData("Shooter Zone", shooterPowerAngleCalculator.getZone());
        //telemetry.addData("turret rotation in degrees", turret.getTurretAngle());
        telemetry.addLine("Turret-----------------------------------");
        telemetry.addData("turret target angle", turret.getTargetAngle());
        telemetry.addData("turret drive angle", turret.getTurretDriveAngle());
        telemetry.addData("turret motor angle", turret.getTurretMotorAngle());
        telemetry.addData("motor PIDF coefficient", robot.turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("current motor tick", turret.getCurrentTick());
        telemetry.addData("target motor tick", turret.getTargetTick());
        telemetry.addLine("-----------------------------------------");
        telemetry.addData("limelight output", "%,.1f",limelight.normalizedPose2D(DistanceUnit.INCH));
        telemetry.addData("limelight angle Tx", limelight.getTargetX());
        telemetry.update();
    }
    public void telemetryManagerSimplified() {
        telemetry.addLine("-----SPINDEXER-----");
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addLine("-----ROBOT STATE-----");
        telemetry.addData("Action State", actionStates);
        telemetry.addData("Requested", requestedActionState);
        telemetry.addData("Active", activeActionState);
        telemetry.addData("IntakeState", FSMIntake.intakeStates);
        telemetry.addData("ShooterState", FSMShooter.shooterState);
        telemetry.addData("IntakeSafe", FSMIntake.canExit());
        telemetry.addData("ShooterSafe", FSMShooter.canExit());
        telemetry.addData("distance to goal", shooterPowerAngleCalculator.getDistance());
        String MotifEnabled;
        String MotifAvailable;
        telemetry.addLine("-----SHOOTER STATE-----");
        telemetry.addData("Shooter Target Colour", FSMShooter.targetColour.name());
        telemetry.addData("power set point-NORMED", FSMShooter.getPower_setpoint());
        telemetry.addData("Shooter Power-LUT OUT", robot.topShooterMotor.getPower());
    }

}

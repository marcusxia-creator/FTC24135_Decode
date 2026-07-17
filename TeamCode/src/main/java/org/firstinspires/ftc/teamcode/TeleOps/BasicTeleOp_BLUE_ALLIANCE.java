package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.FSMIntake.IntakeStates;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SHOOTERSTATE;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.SHOOTER_RPM_CONVERSION;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueAllianceResetPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.kickerRetract;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redAllianceResetPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.BallColor;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;

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
@TeleOp(name = "---------BLUE--🐧----------", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp_BLUE_ALLIANCE extends OpMode {
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
    //private TurretUpd turret;
    private Turret turret;
    public Limelight limelight;
    /// ----------------------------------------------------------------
    // For shooter power and angle calculator
    private LUTPowerCalculator shooterPowerAngleCalculator;

    // for ball colour and colour detection
    private BallColor ballColor;

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
        //robot.initExternalIMU();            //Initialize external IMU

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
        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        limelight = new Limelight(robot);
        limelight.initLimelight(25);
        limelight.start();

        /// 3. turret---------------------------------------------------------------
        //turret = new TurretUpd(robot);
        turret = new Turret(robot, false);

        /// 4.1. power calculator for shooter------------------------------------------------------------
        shooterPowerAngleCalculator = new LUTPowerCalculator(robot);

        /// 4. shooter-------------------------------------------------------------
        FSMShooter = new FSMShooter(gamepadCo1, gamepadCo2, robot, shooterPowerAngleCalculator, gamepadComboInput, turret, limelight);
        FSMShooter.Init();

        /// 5. intake------------------------------------------------------------
        FSMIntake = new FSMIntake(gamepadCo1, gamepadCo2, robot);

        /// 7. alliance selection-----------------------------------------------------------
        alliance = Alliance.BLUE_ALLIANCE;
        shooterPowerAngleCalculator.setAlliance(false);

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
        ballColor = BallColor.fromHue(robot.slotSensor1.getColourHSV()[0]);
        updateLoopFrequency();


        // =========================================================
        // 3. DRIVE (always responsive)
        // =========================================================
        robotDrive.DriveLoop();

        // =========================================================
        // 4. NEW! - ACTION STATE TRANSITION MANAGER (GRACEFUL)
        // =========================================================
        updateActionStateTransitions();

        // =========================================================
        // FIXME: 5. MODIFIED - PER-ACTION "EXTRAS" (NO FSM STATE FORCING HERE)
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
        // 6. ZONE STATUS
        // =========================================================
        int zone = shooterPowerAngleCalculator.getZone();
        turret.updateZoneForGoalPose(zone);
        FSMShooter.updateZoneForGoalPose(zone);

        //Turret PIDF Config
        turret.updatePidFromDashboard();

        // =========================================================
        // 7. SUBSYSTEM FSMs (ALWAYS RUN)
        // =========================================================
        FSMIntake.loop();
        FSMShooter.SequenceShooterLoop();

        // =========================================================
        // 8. LED STATUS (non-blocking)
        // =========================================================
        updateLED();

        // =========================================================
        // 9. TELEMETRY
        // =========================================================
        //telemetryManager();
        telemetry.addData("loopFreq", loopHz);
        telemetry.update();
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
        boolean dpDown =
                ((gamepadCo1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_DOWN))
                        && isButtonDebounced());
        boolean turretReset =
                ((gamepadComboInput.getDriverBackSinglePressed()));

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
    private void updateLED() {

        if (shooterPowerAngleCalculator.getZone() == 0 && Math.abs(turret.getTargetTick() - turret.getCurrentTick())<10 && !limelight.llresult()) {
            //Distance outside shooting zone and no limelight, white alert
            robot.LED.setPosition(1.0); // white colour
        } else if (shooterPowerAngleCalculator.getZone() == 0 && limelight.llresult() && Math.abs(turret.getTargetTick() - turret.getCurrentTick())<10){
            robot.LED.setPosition(0.333); // orange
        } else if (shooterPowerAngleCalculator.getZone() > 0 && limelight.llresult() && Math.abs(turret.getTargetTick() - turret.getCurrentTick())<10){
            robot.LED.setPosition(0.5); // green
        } else if (shooterPowerAngleCalculator.getZone() > 0 && limelight.llresult() && Math.abs(turret.getTargetTick() - turret.getCurrentTick())>10) {
            robot.LED.setPosition(0.388); // yellow
        } else if (shooterPowerAngleCalculator.getZone() > 0 && !limelight.llresult() && Math.abs(turret.getTargetTick() - turret.getCurrentTick())<10) {
            robot.LED.setPosition(0.288); // red
        } else { //Default black
            robot.LED.setPosition(0.0);
        }
    }


    private int updateZone () {
        return shooterPowerAngleCalculator.getZone();
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
        telemetry.addData("Slot 0", robot.slotSensor1.checkBall());
        telemetry.addData("Slot 1", robot.slotSensor2.checkBall());
        telemetry.addData("Slot 2", robot.slotSensor3.checkBall());
        telemetry.addData("Current Pos", robot.spindexerServo.getPosition());
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
        telemetry.addData("Alliance", alliance);
        telemetry.addData("current angle", robot.pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pose2D", robot.pinpoint.getPosition());
        telemetry.addData("Starting Pose",PoseStorage.currentPose);
        Pose2D pose = robot.pinpoint.getPosition();
        double xIn = pose.getX(DistanceUnit.INCH);
        double yIn = pose.getY(DistanceUnit.INCH);
        double headingDeg = Math.toDegrees(pose.getHeading(AngleUnit.RADIANS));
        telemetry.addData(
                "Pose (in)",
                "X: %.2f  Y: %.2f  H: %.1f°",
                xIn, yIn, headingDeg
        );
        telemetry.addData("Starting Pose",PoseStorage.currentPose);


        telemetry.addData("distance to goal", "%,.0f",shooterPowerAngleCalculator.getDistance());
        telemetry.addData("Shooter Zone", shooterPowerAngleCalculator.getZone());
        telemetry.addLine("Turret-----------------------------------");
        telemetry.addData("turret target angle", turret.getTargetAngle());
        telemetry.addData("turret drive angle", turret.getTurretDriveAngle());
        telemetry.addData("turret motor angle", turret.getTurretMotorAngle());
        telemetry.addData("motor PIDF coefficient", robot.turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addData("current motor tick", turret.getCurrentTick());
        telemetry.addData("target motor tick", turret.getTargetTick());
        telemetry.addData("goal pose", turret.getGoalPose());
        telemetry.addLine("-----------------------------------------");
        ///telemetry.addData("limelight output", "%,.1f",limelight.normalizedPose2D(DistanceUnit.INCH));
        telemetry.addData("limelight angle Tx", limelight.getTargetXForTag(25));
        telemetry.addData("green slot position", limelight.getGreenSlot());
        telemetry.update();
    }
    public void telemetryManagerSimplified() {
        telemetry.addLine("-----SPINDEXER-----");
        telemetry.addData("Slot 0", robot.slotSensor1.checkBall());
        telemetry.addData("Slot 1", robot.slotSensor2.checkBall());
        telemetry.addData("Slot 2", robot.slotSensor3.checkBall());
        telemetry.addLine("-----ROBOT STATE-----");
        telemetry.addData("Action State", actionStates);
        telemetry.addData("Requested", requestedActionState);
        telemetry.addData("Active", activeActionState);
        telemetry.addData("IntakeState", FSMIntake.intakeStates);
        telemetry.addData("ShooterState", FSMShooter.shooterState);
        telemetry.addData("IntakeSafe", FSMIntake.canExit());
        telemetry.addData("ShooterSafe", FSMShooter.canExit());
        telemetry.addData("distance to goal", shooterPowerAngleCalculator.getDistance());
        telemetry.addLine("-----SHOOTER STATE-----");
        telemetry.addData("power set point-NORMED", FSMShooter.getPower_setpoint());
        telemetry.addData("Shooter Power-LUT OUT", robot.topShooterMotor.getPower());
    }

}

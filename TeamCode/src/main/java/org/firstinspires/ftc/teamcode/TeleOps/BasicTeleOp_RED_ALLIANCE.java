package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.FSMIntake.IntakeStates;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SHOOTERSTATE;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueAllianceResetPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redAllianceResetPose;

import com.acmerobotics.dashboard.FtcDashboard;
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
    /// For alliance colour
    public static Alliance alliance;

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

        Pose2D startingPose = new Pose2D(DistanceUnit.INCH, PoseStorage.currentPose.position.x, PoseStorage.currentPose.position.y, AngleUnit.RADIANS, PoseStorage.currentPose.heading.real);
        robot.pinpoint.setPosition(startingPose);

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
    }

    @Override
    public void loop() {
        /// 1. Buttons--------------------------------------------------------------
        /// Read gamepad buttons for wasJustPressed events
        gamepadCo1.readButtons();
        gamepadCo2.readButtons();

        /// combo button LB+ & RB+ config and update
        gamepadInput.update();
        /// Changes the action state base on which button is pressed
        buttonUpdate();

        /// 2. Continuous updates -pinpoint & ballcolor & loop frequency--------------------
        robot.pinpoint.update();
        ballColor = BallColor.fromHue(colorDetection.getHue());
        updateLoopFrequency();


        /// 3. Subsystem Continous Running----------------------------------------------
        robotDrive.DriveLoop();
        ///FSM intake controller
        FSMIntake.loop();

        /// Robot Action States
        switch (actionStates){
            case Sequence_Shooting:
                FSMShooter.SequenceShooterLoop();
                turret.driveTurretMotor();
                FSMIntake.intakeStates = IntakeStates.INTAKE_IDLE;
                break;
            case Sort_Shooting:
                //FSMShooter.SortShooterLoop();
                break;
            case Intaking:
                FSMShooter.shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                break;
            case Idle:
                FSMIntake.intakeStates = IntakeStates.INTAKE_IDLE;
                FSMShooter.shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                break;
        }
        /// 4. LED alarm light ----------------------------------------------
        if (shooterPowerAngleCalculator.getZone() ==0) {
            //Distance less than 54 inches, red alert
            robot.LED.setPosition(0.28);
        }
        else if (shooterPowerAngleCalculator.getZone() > 0){
            robot.LED.setPosition(1.0);
        }
        else { //Default white
            robot.LED.setPosition(0.0);
        }
        /// 5. telemetry helper ----------------------------------------------
        telemetryManager();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    /**
     * Button debounce helper
     */
    public boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    /// Frequency Updates
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

    /// telemetry Manager
    public void telemetryManager(){
        telemetry.addData("Action State", actionStates);
        telemetry.addLine("-----");
        telemetry.addData("Intake State", FSMIntake.intakeStates);
        telemetry.addData("Distance Sensor", robot.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Sensor Color", colorDetection.getStableColor());
        telemetry.addData("Sensor values", spindexer.colorValue);
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Pos", spindexer.currentPos);
        telemetry.addData("Shooter Target Colour", FSMShooter.targetColour.name());
        telemetry.addLine("-----");
        telemetry.addData("Shooter State", FSMShooter.shooterState);
        telemetry.addData("shooter power calculator", shooterPowerAngleCalculator.getPower());
        telemetry.addData("shooter power from FSM Shooter", FSMShooter.getPower());
        telemetry.addData("voltage from Shooter", FSMShooter.getVoltage());
        telemetry.addData("voltage from robot", robot.getBatteryVoltageRobust());
        telemetry.addData("power set point", FSMShooter.getPower_setpoint());
        telemetry.addData("Shooter Power", robot.topShooterMotor.getPower());
        telemetry.addData("Shooter Velocity", robot.topShooterMotor.getVelocity());
        telemetry.addData("Shooter Motor Mode", robot.topShooterMotor.getMode());
        telemetry.addLine("-----");
        String MotifAvailable;
        telemetry.addData("current angle", robot.pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose2D", robot.pinpoint.getPosition());
        telemetry.addData("distance to goal", shooterPowerAngleCalculator.getDistance());
        telemetry.addData("Shooter Zone", shooterPowerAngleCalculator.getZone());
        //telemetry.addData("turret rotation in degrees", turret.getTurretAngle());
        telemetry.addLine("Turret-----------------------------------");
        telemetry.addData("turret target angle", turret.getTargetAngle());
        telemetry.addData("turret drive angle", turret.getTurretDriveAngle());
        telemetry.addData("turret motor angle", turret.getTurretMotorAngle());
        telemetry.addData("motor PIDF coefficient", robot.turretMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        telemetry.addLine("-----------------------------------------");
        telemetry.addData("limelight output", limelight.normalizedPose2D(DistanceUnit.MM));
        telemetry.update();
    }
    public void telemetryManagerSimplified() {
        telemetry.addLine("-----SPINDEXER-----");
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addLine("-----SHOOTER-----");
        telemetry.addData("Shooter State", FSMShooter.shooterState);
        String MotifEnabled;
        String MotifAvailable;

        telemetry.addData("Shooter Target Colour", FSMShooter.targetColour.name());
        telemetry.addData("power set point", FSMShooter.getPower_setpoint());
        telemetry.addData("Shooter Power", robot.topShooterMotor.getPower());
        telemetry.addLine("-----ROBOT-----");
        telemetry.addData("distance to goal", shooterPowerAngleCalculator.getDistance());
        telemetry.addLine("-----INTAKE-----");
        telemetry.addData("Intake State", FSMIntake.intakeStates);
    }

    private void buttonUpdate() {
        //Button x - For sequence shooting
        if (gamepadCo1.getButton(GamepadKeys.Button.X) || gamepadCo2.getButton(GamepadKeys.Button.X)
                && isButtonDebounced()){
            actionStates = RobotActionState.Sequence_Shooting;
            FSMShooter.shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
            FSMIntake.intakeStates = IntakeStates.INTAKE_STOP;
        }

        //Button y - For sort shooting
        if (gamepadCo1.getButton(GamepadKeys.Button.Y) || gamepadCo2.getButton(GamepadKeys.Button.Y)
                && isButtonDebounced()){
           // actionStates = RobotActionState.Sort_Shooting;
            FSMIntake.intakeStates = IntakeStates.INTAKE_STOP;
            //FSMShooter.sortShooterState = SORTSHOOTERSTATE.SHOOTER_IDLE;
        }

        //Dpad left - For intaking
        if (gamepadCo1.getButton(GamepadKeys.Button.DPAD_LEFT) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_LEFT)
                && isButtonDebounced()){
            actionStates = RobotActionState.Intaking;
            FSMIntake.intakeStates = IntakeStates.INTAKE_PREP;
        }

        //Dpad right - For reversing intake
        if (gamepadCo1.getButton(GamepadKeys.Button.DPAD_RIGHT) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_RIGHT)
                && isButtonDebounced()) {
            FSMIntake.intakeTimer.reset();
            FSMIntake.reversing();
        }

        //Button B - idle state
        if (gamepadCo1.getButton(GamepadKeys.Button.B) || gamepadCo2.getButton(GamepadKeys.Button.B)
                && isButtonDebounced()) {
            actionStates = RobotActionState.Idle;
        }

        //Dpad down for alliance selection
        if (gamepadCo1.getButton(GamepadKeys.Button.DPAD_DOWN) || gamepadCo2.getButton(GamepadKeys.Button.DPAD_DOWN)) {
            //Reset robot red alliance pose
            if (alliance == Alliance.RED_ALLIANCE) {
                robot.pinpoint.setPosition(redAllianceResetPose);
                robot.LED.setPosition(0.28);
            }
            //Reset robot blue alliance pose
            if (alliance == Alliance.BLUE_ALLIANCE) {
                robot.pinpoint.setPosition(blueAllianceResetPose);
                robot.LED.setPosition(0.611);
            }
        }
    }
}

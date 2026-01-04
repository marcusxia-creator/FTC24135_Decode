package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.AprilTagMotif.MotifMemorization.motif;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMIntake.IntakeStates;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SHOOTERSTATE;
import static org.firstinspires.ftc.teamcode.TeleOps.FSMShooter.SORTSHOOTERSTATE;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueAllianceResetPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.close;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.farEdge;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redAllianceResetPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.Tests.BallColor;

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
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private GamepadInput gamepadInput;
    private RobotDrive robotDrive;
    FSMShooter FSMShooter;
    FSMIntake FSMIntake;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private Turret turret;
    /// ----------------------------------------------------------------
    /**
     * Need to determine the following section later.
    */
    private SpindexerManualControl spindexerManualControl;

    private Spindexer spindexer;

    private LUTPowerCalculator shooterPowerAngleCalculator;

    private static double voltage;
    private BallColor ballColor;
    private ColorDetection colorDetection;
    /// ----------------------------------------------------------------
    /// For robot action state
    public RobotActionState actionStates;
    /// For alliance colour
    public static Alliance alliance;


    @Override
    public void init() {
        /// For telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /// For robot hardware initialization
        robot = new RobotHardware(hardwareMap);
        robot.init(); //Initialize all motors and servos
        robot.initIMU(); //Initialize control hub IMU
        robot.initPinpoint(); //Initialize pinpoint
        robot.initExternalIMU(); //Initialize external IMU

        /// ---------------------------------------------------------------
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        gamepadInput = new GamepadInput(gamepadCo1,gamepadCo2);
        spindexer = new Spindexer(robot, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, 0); //Change inits for comp
        spindexer.runToSlot(0);
        spindexerManualControl = new SpindexerManualControl(robot, spindexer, gamepadInput);
        /// ---------------------------------------------------------------
        turret = new Turret(robot);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        shooterPowerAngleCalculator = new LUTPowerCalculator(robot);
        colorDetection = new ColorDetection(robot);

        alliance = Alliance.RED_ALLIANCE;
        shooterPowerAngleCalculator.setAlliance(true);

        FSMShooter = new FSMShooter(gamepadCo1, gamepadCo2, robot, spindexer, shooterPowerAngleCalculator,gamepadInput);
        FSMShooter.Init();

        FSMIntake = new FSMIntake(gamepadCo1, gamepadCo2, robot, spindexer);

        actionStates = RobotActionState.Idle;
    }

    @Override
    public void loop() {
        //Read gamepad buttons for wasJustPressed events
        gamepadCo1.readButtons();
        gamepadCo2.readButtons();

        gamepadInput.update();
        robot.pinpoint.update();
        ballColor = BallColor.fromHue(colorDetection.getHue());

        //Always allow manual control and driving
        spindexerManualControl.loop();
        robotDrive.DriveLoop();

        switch (actionStates){
            case Sequence_Shooting:
                FSMShooter.SequenceShooterLoop();
                FSMShooter.shooterState = SHOOTERSTATE.FLYWHEEL_RUNNING;
                break;
            case Sort_Shooting:
                FSMShooter.SortShooterLoop();
                FSMShooter.sortShooterState = SORTSHOOTERSTATE.FLYWHEEL_RUNNING;
                break;
            case Intaking:
                FSMIntake.loop();
                FSMIntake.intakeStates = IntakeStates.INTAKE_START;
                break;
            case Idle:
                FSMIntake.intakeStates = IntakeStates.INTAKE_IDLE;
                FSMShooter.shooterState = SHOOTERSTATE.SHOOTER_IDLE;
                break;
        }

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
        //LED alarm light
        if (shooterPowerAngleCalculator.getDistance() <= close||shooterPowerAngleCalculator.getDistance() >= farEdge) {
            //Distance less than 54 inches, red alert
            robot.LED.setPosition(0.28);
        }
        else if (ballColor.isKnown()) { //Show green and purple colour
            if (ballColor == BallColor.GREEN) {
                robot.LED.setPosition(0.5);
            }
            if (ballColor == BallColor.PURPLE) {
                robot.LED.setPosition(0.722);
            }
        }
        else { //Default white
            robot.LED.setPosition(1.0);
        }
        telemetryManager();
    }

    @Override
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
    public boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }

    public void telemetryManager(){
        telemetry.addData("Action State", actionStates);
        telemetry.addData("Intake State", FSMIntake.intakeStates);
        telemetry.addData("Sensor Distance", robot.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Sensor Color", colorDetection.getStableColor());
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Slot", spindexer.currentSlot);
        telemetry.addData("Shooter Target Colour", FSMShooter.targetColour.name());
        if(motif!=null) {
            telemetry.addData("Motif Green Count", motif.countFrom(Spindexer.SLOT.Green, spindexer.count(Spindexer.SLOT.Empty)));
            telemetry.addData("Motif Purple Count", motif.countFrom(Spindexer.SLOT.Purple, spindexer.count(Spindexer.SLOT.Empty)));
        }
        telemetry.addLine("-----");
        telemetry.addData("Shooter State", FSMShooter.shooterState);
        telemetry.addData("shooter power calculator", shooterPowerAngleCalculator.getPower());
        telemetry.addData("shooter speed from FSM Shooter", FSMShooter.getSpeed());
        telemetry.addData("voltage from Shooter", FSMShooter.getVoltage());
        telemetry.addData("voltage from robot", robot.getBatteryVoltageRobust());
        telemetry.addData("power set point", FSMShooter.getPower_setpoint());
        telemetry.addData("Shooter Power", robot.topShooterMotor.getPower());
        telemetry.addData("Shooter Velocity", robot.topShooterMotor.getVelocity());
        telemetry.addData("Shooter Motor Mode", robot.topShooterMotor.getMode());
        telemetry.addLine("-----");
        String MotifAvailable;
        if (spindexer.checkMotif(motif)) {MotifAvailable = "Available";} else {MotifAvailable = "Not Available";}
        if(motif==null){
            telemetry.addData("Motif","null");
        }
        else{
            telemetry.addData("Motif",motif.name);
        }
        telemetry.addData("desired angle", shooterPowerAngleCalculator.getAngle());
        telemetry.addData("desired robot angle", 90 + shooterPowerAngleCalculator.getAngle()); //If the desired robot angle equal to the current angle, then the robot is on course
        telemetry.addData("current angle", robot.pinpoint.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Pose2D", robot.pinpoint.getPosition());
        telemetry.addData("distance to goal", shooterPowerAngleCalculator.getDistance());
        telemetry.addData("turret rotation in degrees", turret.getTurretAngle());
        telemetry.addData("turret target angle", turret.getTargetAngle());
        telemetry.update();
    }
    public void telemetryManagerSimplified() {
        telemetry.addLine("-----SPINDEXER-----");
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Slot", spindexer.currentSlot);
        telemetry.addLine("-----SHOOTER-----");
        telemetry.addData("Shooter State", FSMShooter.shooterState);
        String MotifEnabled;
        String MotifAvailable;
        if (spindexer.checkMotif(motif)) {
            MotifAvailable = "Available";
        } else {
            MotifAvailable = "Not Available";
        }

        telemetry.addData("Shooter Target Colour", FSMShooter.targetColour.name());
        telemetry.addData("power set point", FSMShooter.getPower_setpoint());
        telemetry.addData("Shooter Power", robot.topShooterMotor.getPower());
        telemetry.addLine("-----ROBOT-----");
        telemetry.addData("distance to goal", shooterPowerAngleCalculator.getDistance());
        telemetry.addLine("-----INTAKE-----");
        telemetry.addData("Intake State", FSMIntake.intakeStates);
    }
}

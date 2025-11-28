package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.MotifMemorization.motif;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueAllianceResetPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.close;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.closeEdge;
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
import org.firstinspires.ftc.teamcode.TeleOps.Tests.BallColor;
import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;

@TeleOp (name = "RED_TELEOP_MEET_2", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp_RED_ALLIANCE extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private GamepadInput gamepadInput;
    private RobotDrive robotDrive;
    private FSMShooter FSMShooter;
    private FSMIntake FSMIntake;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private SpindexerManualControl spindexerManualControl;

    private GamepadManager gamepadManager;
    private Spindexer spindexer;

    private LUTPowerCalculator shooterPowerAngleCalculator;

    private static double voltage;
    private BallColor ballColor;
    private ColorDetection colorDetection;

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public static Alliance alliance;


    @Override
    public void init() {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        gamepadManager = new GamepadManager(gamepad1,gamepad2);
        gamepadInput = new GamepadInput(gamepadCo1,gamepadCo2);
        spindexer = new Spindexer(robot, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, 0); //Change inits for comp
        spindexer.runToSlot(0);
        spindexerManualControl = new SpindexerManualControl(robot, spindexer, gamepadManager, gamepadInput);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        shooterPowerAngleCalculator = new LUTPowerCalculator(robot);
        colorDetection = new ColorDetection(robot);

        alliance = Alliance.RED_ALLIANCE;
        shooterPowerAngleCalculator.setAlliance(true);

        FSMShooter = new FSMShooter(gamepadCo1, gamepadCo2, robot, spindexer, gamepadManager, shooterPowerAngleCalculator,gamepadInput);
        FSMShooter.Init();

        FSMIntake = new FSMIntake(gamepadCo1, gamepadCo2, robot, spindexer, gamepadManager);
    }

    @Override
    public void loop() {
        robot.pinpoint.update();
        ballColor = BallColor.fromHue(colorDetection.getHue());
        gamepadManager.loop();
        FSMShooter.ShooterLoop();
        FSMIntake.loop();
        //?? need comments
        spindexer.runToSlot();
        spindexerManualControl.loop();
        robotDrive.DriveLoop();


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
        telemetry.addData("Intake State", FSMIntake.intakeStates);
        telemetry.addData("Sensor Distance", robot.distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("Sensor Color", colorDetection.getStableColor());
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Slot", spindexer.currentSlot);
        telemetry.addData("Spindexer Servo Pos", robot.spindexerServo.getPosition());
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
        telemetry.addData("Shooter Power", robot.shooterMotor.getPower());
        telemetry.addData("Shooter Velocity", robot.shooterMotor.getVelocity());
        telemetry.addData("Shooter Motor Mode", robot.shooterMotor.getMode());
        telemetry.addData("Shooter Power Mode", FSMShooter.shooterpowerstate);
        telemetry.addLine("-----");
        String MotifEnabled;
        if (gamepadManager.autoMotif.ToggleState) {MotifEnabled = "Enabled";} else {MotifEnabled = "Disabled";}
        String MotifAvailable;
        if (spindexer.checkMotif(motif)) {MotifAvailable = "Available";} else {MotifAvailable = "Not Available";}
        telemetry.addData("Auto Motif",String.join(", ",MotifEnabled, MotifAvailable));
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
        telemetry.update();
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
}

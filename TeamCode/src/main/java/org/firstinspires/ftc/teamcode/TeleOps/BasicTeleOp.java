package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp (name = "Basic TeleOp", group = "org.firstinspires.ftc.teamcode")
public class BasicTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepadCo1, gamepadCo2;
    private RobotDrive robotDrive;
    private FSMShooter shooterManualControl;
    private Intake intake;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private SpindexerManualControl spindexerManualControl;

    private GamepadManager gamepadManager;
    private Spindexer spindexer;

    private ShooterPowerCalculator shooterPowerCalculator;

    private static double voltage;
    private BallColor ballColor;
    private ColorDetection colorDetection;

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public static Alliance alliance;

    private final Pose2D blueAllianceResetPose = new Pose2D(DistanceUnit.INCH, -72, -72, AngleUnit.DEGREES, 0);
    private final Pose2D redAllianceResetPose = new Pose2D(DistanceUnit.INCH, 72, -72, AngleUnit.DEGREES, 0);


    @Override
    public void init() {
        gamepadCo1 = new GamepadEx(gamepad1);
        gamepadCo2 = new GamepadEx(gamepad2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        gamepadManager= new GamepadManager(gamepad1,gamepad2);
        spindexer = new Spindexer(robot, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, Spindexer.SLOT.Empty, 0); //Change inits for comp
        spindexer.runToSlot(0);
        spindexerManualControl = new SpindexerManualControl(robot, spindexer, gamepadManager);

        robotDrive = new RobotDrive(robot, gamepadCo1, gamepadCo2);
        robotDrive.Init();

        shooterManualControl = new FSMShooter(gamepadCo1, gamepadCo2, robot, spindexer, gamepadManager);
        shooterManualControl.Init();

        intake = new Intake(gamepadCo1, gamepadCo2, robot, spindexer, gamepadManager);

        shooterPowerCalculator = new ShooterPowerCalculator(robot);
    }

    @Override
    public void loop() {
        robot.pinpoint.update();

        //robotDrive.DriveLoop();
        gamepadManager.loop();

        shooterManualControl.ShooterLoop();
        intake.loop();

        spindexer.runToSlot();
        spindexerManualControl.loop();

        robotDrive.DriveLoop();

        if (shooterPowerCalculator.getDistance() <= 54) {
            robot.LED.setPosition(0.28);
        }
        else if (ballColor.isKnown()) {
            if (ballColor == BallColor.GREEN) {
                robot.LED.setPosition(0.5);
            }
            if (ballColor == BallColor.PURPLE) {
                robot.LED.setPosition(0.722);
            }
        }
        else {
            robot.LED.setPosition(1.0);
        }

        if ((gamepadCo1.getButton(GamepadKeys.Button.START) && gamepadCo1.getButton(GamepadKeys.Button.LEFT_BUMPER)) || (gamepadCo2.getButton(GamepadKeys.Button.START) && gamepadCo2.getButton(GamepadKeys.Button.LEFT_BUMPER))) {
            alliance = Alliance.RED_ALLIANCE;
            shooterPowerCalculator.setAlliance(true);
            robot.LED.setPosition(0.28);
        }
        if ((gamepadCo1.getButton(GamepadKeys.Button.START) && gamepadCo1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) || (gamepadCo2.getButton(GamepadKeys.Button.START) && gamepadCo2.getButton(GamepadKeys.Button.RIGHT_BUMPER))) {
            alliance = Alliance.BLUE_ALLIANCE;
            shooterPowerCalculator.setAlliance(false);
            robot.LED.setPosition(0.611);
        }

        if (gamepadCo1.getButton(GamepadKeys.Button.BACK) || gamepadCo2.getButton(GamepadKeys.Button.BACK)) {
            if (alliance == Alliance.RED_ALLIANCE) {
                robot.pinpoint.setPosition(redAllianceResetPose);
                robot.LED.setPosition(0.28);
            }

            if (alliance == Alliance.BLUE_ALLIANCE) {
                robot.pinpoint.setPosition(blueAllianceResetPose);
                robot.LED.setPosition(0.611);
            }
        }
        telemetry.addData("Intake State", intake.intakeStates);
        telemetry.addData("Sensor Distance", robot.distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("Slot 0", spindexer.slots[0]);
        telemetry.addData("Slot 1", spindexer.slots[1]);
        telemetry.addData("Slot 2", spindexer.slots[2]);
        telemetry.addData("Current Slot", spindexer.currentSlot);
        telemetry.addData("Spindexer Servo Pos", robot.spindexerServo.getPosition());
        telemetry.addData("Target Colour", shooterManualControl.targetColour.name());
        telemetry.addLine("-----");
        telemetry.addData("Shooter State", shooterManualControl.shooterState);
        telemetry.addData("Shooter Power", robot.shooterMotor.getPower());
        telemetry.addData("Shooter Velocity", robot.shooterMotor.getVelocity());
        telemetry.addData("Shooter Motor Mode", robot.shooterMotor.getMode());
        telemetry.addData("Shooter dI", shooterManualControl.dt);
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

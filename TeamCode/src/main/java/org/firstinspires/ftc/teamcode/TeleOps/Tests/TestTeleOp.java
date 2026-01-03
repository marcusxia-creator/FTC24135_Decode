package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeleOps.Sensors.ColorDetection;
import org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig;
import org.firstinspires.ftc.teamcode.TeleOps.RobotDrive;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.ShooterPowerAngleCalculator;

@Config
@TeleOp (name = "TestTeleOp", group = "org.firstinspires.ftc.teamcode")
public class TestTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private double servoposition;
    private static double speed;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private RobotDrive robotDrive;
    private ShooterPowerAngleCalculator shooterPowerAngleCalculator;

    private static double voltage;
    private BallColor ballColor;
    private ColorDetection colorDetection;

    double intakeSpeed = 0.5;
    double depositSpeed = 0.7;


    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        robotDrive = new RobotDrive(robot, gamepad_1, gamepad_2);
        robotDrive.Init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servoposition = 0.0;
        speed = 0.0;

        shooterPowerAngleCalculator = new ShooterPowerAngleCalculator(robot);
        robotDrive = new RobotDrive(robot, gamepad_1, gamepad_2);

        robot.leftSpindexerServo.setPosition(RobotActionConfig.spindexerReset);

        colorDetection = new ColorDetection(robot);

        //gamepadManager=new GamepadManager(gamepad1,gamepad2);
    }

    @Override
    public void loop() {
        robot.pinpoint.update();
        voltage = robot.getBatteryVoltageRobust();
        speed = shooterPowerAngleCalculator.getPower();
        double power_setpoint = speed*12.0/voltage;

        robotDrive.DriveLoop();
        ballColor = BallColor.fromHue(colorDetection.getHue());

        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            servoposition = robot.leftSpindexerServo.getPosition() + 0.01;
            servoposition = robot.rightSpindexerServo.getPosition() + 0.01;
            robot.leftSpindexerServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.rightSpindexerServo.setPosition(Range.clip(servoposition, 0, 1));

        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            servoposition = robot.leftSpindexerServo.getPosition() - 0.01;
            servoposition = robot.rightSpindexerServo.getPosition() - 0.01;
            robot.leftSpindexerServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.rightSpindexerServo.setPosition(Range.clip(servoposition, 0, 1));

        }

        if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()){
            //speed = robot.shooterMotor.getPower() + 0.05;
            //robot.shooterMotor.setPower(Range.clip(/*power_setpoint*/ 0.7,0.3,1.0));
            robot.topShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.shooterMotor.setPower(Range.clip(depositSpeed, 0.3, 1.0));\
            robot.topShooterMotor.setPower(0.7);
            depositSpeed += 0.05;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            robot.topShooterMotor.setPower(0);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()) {
            robot.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            speed = robot.intakeMotor.getPower() + 0.05;
            //robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //robot.intakeMotor.setPower(Range.clip(speed,0.5,1.0));
            robot.intakeMotor.setPower(Range.clip(intakeSpeed, 0.5, 1.0));
            intakeSpeed += 0.05;
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
        }

        if (shooterPowerAngleCalculator.getDistance() <= 54) {
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
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("Left Spindexer Position", robot.leftSpindexerServo.getPosition());
        telemetry.addData("Right Spindexer Position", robot.rightSpindexerServo.getPosition());
        telemetry.addData("Shooter Speed", robot.topShooterMotor.getPower());
        telemetry.addData("Intake Speed", robot.intakeMotor.getPower());
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("Pose 2D", robot.pinpoint.getPosition());
        telemetry.addData("Distance To Goal", shooterPowerAngleCalculator.getDistance());
        telemetry.addData("Robot Voltage", robot.getBatteryVoltageRobust());
        telemetry.addData("Shooter Power Setpoint", speed);
        telemetry.addData("Shooter Actual Power Setpoint", power_setpoint);
        telemetry.addData("Shooter Motor Power Reading", robot.topShooterMotor.getPower());
        telemetry.addData("Shooter Motor Power Calculator", shooterPowerAngleCalculator.getPower());
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("Color", ballColor);

        telemetry.update();
    }

    private boolean isButtonDebounced() {
        if (debounceTimer.seconds() > RobotActionConfig.DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            return true;
        }
        return false;
    }
}



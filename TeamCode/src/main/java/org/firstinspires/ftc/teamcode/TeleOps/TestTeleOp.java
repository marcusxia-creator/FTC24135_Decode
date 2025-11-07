package org.firstinspires.ftc.teamcode.TeleOps;

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
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

import android.graphics.Color;

@Config
@TeleOp (name = "TestTeleOp", group = "org.firstinspires.ftc.teamcode")
public class TestTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private double servoposition;
    public static double speed;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private RobotDrive robotDrive;
    private ShooterPowerCalculator shooterPowerCalculator;

    private static double voltage;
    private BallColor ballColor;
    private ColorDetection colorDetection;


    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        robotDrive = new RobotDrive(robot, gamepad_1, gamepad_2, shooterPowerCalculator);
        robotDrive.Init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servoposition = 0.0;
        speed = 0.0;

        shooterPowerCalculator = new ShooterPowerCalculator(robot);
        robotDrive = new RobotDrive(robot, gamepad_1, gamepad_2, shooterPowerCalculator);

        robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
        robot.leftGateServo.setPosition(RobotActionConfig.gateUp);
        robot.rightGateServo.setPosition(RobotActionConfig.gateUp);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerReset);

        colorDetection = new ColorDetection(robot);

        //gamepadManager=new GamepadManager(gamepad1,gamepad2);
    }

    @Override
    public void loop() {
        robot.pinpoint.update();
        voltage = robot.getBatteryVoltageRobust();
        speed = shooterPowerCalculator.getPower();
        double power_setpoint = speed*12.0/voltage;

        robotDrive.DriveLoop();
        ballColor = BallColor.fromHue(colorDetection.getHue());


        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            //servoposition = robot.pushRampServo.getPosition() + 0.01;
            robot.pushRampServo.setPosition(Range.clip(rampDownPos, 0.0, 1.0
            ));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            //servoposition = robot.pushRampServo.getPosition() - 0.01;
            robot.pushRampServo.setPosition(Range.clip(rampUpPos, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() + 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()) {
            servoposition = robot.spindexerServo.getPosition() - 0.01;
            robot.spindexerServo.setPosition(Range.clip(servoposition, 0, 1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_UP) && isButtonDebounced()) {
            servoposition = robot.leftGateServo.getPosition() + 0.01;
            robot.leftGateServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.rightGateServo.setPosition(Range.clip(servoposition,0,1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.DPAD_DOWN) && isButtonDebounced()) {
            servoposition = robot.leftGateServo.getPosition() - 0.01;
            robot.leftGateServo.setPosition(Range.clip(servoposition, 0, 1));
            robot.rightGateServo.setPosition(Range.clip(servoposition,0,1));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.X) && isButtonDebounced()){
            //speed = robot.shooterMotor.getPower() + 0.05;
            robot.shooterMotor.setPower(Range.clip(power_setpoint,0.3,1.0));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            robot.shooterMotor.setPower(0);
        }
        if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER) && isButtonDebounced()){
            robot.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            speed = robot.intakeMotor.getPower() + 0.05;
            //robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.intakeMotor.setPower(Range.clip(speed,0.5,1.0));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
        }

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

        telemetry.addData("Ramp Position", robot.pushRampServo.getPosition());
        telemetry.addData("Left Gate Position", robot.leftGateServo.getPosition());
        telemetry.addData("Right Gate Position", robot.rightGateServo.getPosition());
        telemetry.addData("Spindexer Position", robot.spindexerServo.getPosition());
        telemetry.addData("Shooter Speed", robot.shooterMotor.getPower());
        telemetry.addData("Intake Speed", robot.intakeMotor.getPower());
        telemetry.addLine("----------------------------------------------------");
        telemetry.addData("Pose 2D", robot.pinpoint.getPosition());
        telemetry.addData("Distance To Goal", shooterPowerCalculator.getDistance());
        telemetry.addData("Robot Voltage", robot.getBatteryVoltageRobust());
        telemetry.addData("Shooter Power Setpoint", speed);
        telemetry.addData("Shooter Actual Power Setpoint", power_setpoint);
        telemetry.addData("Shooter Motor Power Reading", robot.shooterMotor.getPower());
        telemetry.addData("Shooter Motor Power Calculator", shooterPowerCalculator.getPower());
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



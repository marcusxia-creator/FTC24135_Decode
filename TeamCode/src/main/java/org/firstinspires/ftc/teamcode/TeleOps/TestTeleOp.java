package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name = "TestTeleOp", group = "org.firstinspires.ftc.teamcode")
public class TestTeleOp extends OpMode {
    private RobotHardware robot;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;
    private double servoposition;
    private double speed;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private RobotDrive robotDrive;
    private GamepadManager gamepadManager;

    @Override
    public void init() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);
        robot = new RobotHardware(hardwareMap);
        robot.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servoposition = 0.0;
        speed = 0.0;

        robot.pushRampServo.setPosition(RobotActionConfig.rampDownPos);
        robot.leftGateServo.setPosition(RobotActionConfig.gateDown);
        robot.rightGateServo.setPosition(RobotActionConfig.gateDown);
        robot.spindexerServo.setPosition(RobotActionConfig.spindexerReset);

        gamepadManager=new GamepadManager(gamepad1,gamepad2);
    }

    @Override
    public void loop() {
        if (gamepad_1.getButton(GamepadKeys.Button.A) && isButtonDebounced()) {
            servoposition = robot.pushRampServo.getPosition() + 0.01;
            robot.pushRampServo.setPosition(Range.clip(servoposition, 0.0, 1.0
            ));
        }
        if (gamepad_1.getButton(GamepadKeys.Button.B) && isButtonDebounced()) {
            servoposition = robot.pushRampServo.getPosition() - 0.01;
            robot.pushRampServo.setPosition(Range.clip(servoposition, 0, 1));
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
        if (gamepad_2.getButton(GamepadKeys.Button.X) && isButtonDebounced()){
            speed = robot.shooterMotor.getPower() + 0.05;
            robot.shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.shooterMotor.setPower(Range.clip(speed,0.5,1.0));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.Y) && isButtonDebounced()){
            robot.shooterMotor.setPower(0);
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && isButtonDebounced()){
            robot.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            speed = robot.intakeMotor.getPower() + 0.05;
            robot.intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.intakeMotor.setPower(Range.clip(speed,0.5,1.0));
        }
        if (gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT) && isButtonDebounced()){
            robot.intakeMotor.setPower(0);
        }
        telemetry.addData("Ramp Position", robot.pushRampServo.getPosition());
        telemetry.addData("Left Gate Position", robot.leftGateServo.getPosition());
        telemetry.addData("Right Gate Position", robot.rightGateServo.getPosition());
        telemetry.addData("Spindexer Position", robot.spindexerServo.getPosition());
        telemetry.addData("Shooter Speed", robot.shooterMotor.getPower());
        telemetry.addData("Intake Speed", robot.intakeMotor.getPower());

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



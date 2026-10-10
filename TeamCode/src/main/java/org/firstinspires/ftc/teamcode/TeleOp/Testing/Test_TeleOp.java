package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@Config
@TeleOp (name = "Test Dual PID", group = "Test")
public class Test_TeleOp extends OpMode {
    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx intakeMotor;
    private GamepadEx gamepad;
    private ElapsedTime debounceTimer = new ElapsedTime();
    private final double DEBOUNCE_THRESHOLD = 0.2;
    private FtcDashboard dashboard;

    public static double leftShooterKp = 0.001, leftShooterKi = 0, leftShooterKd = 0;
    public static double rightShooterKp = 0.001, rightShooterKi = 0, rightShooterKd = 0;
    public static double intakeKp = 0.001, intakeKi = 0, intakeKd = 0;

    private PIDController leftShooterPID = new PIDController(leftShooterKp, leftShooterKi, leftShooterKd);
    private PIDController rightShooterPID = new PIDController(rightShooterKp, rightShooterKi, rightShooterKd);
    private PIDController intakePID = new PIDController(intakeKp, intakeKi, intakeKd);

    public static double leftShooterTargetVelocity = 0.0;
    public static double rightShooterTargetVelocity = 0.0;
    public static double intakeTargetVelocity = 0.0;

    @Override
    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "Left_Shooter_Motor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "Right_Shooter_Motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gamepad = new GamepadEx(gamepad1);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop(){

        leftShooterPID.setPID(leftShooterKp, leftShooterKi, leftShooterKd);
        rightShooterPID.setPID(rightShooterKp, rightShooterKi, rightShooterKd);
        intakePID.setPID(intakeKp, intakeKi, intakeKd);

        if (gamepad.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity = 2000.0;
            rightShooterTargetVelocity = 2000.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.Y) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity = 0.0;
            rightShooterTargetVelocity = 0.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity += 100.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            leftShooterTargetVelocity -= 100.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            intakeTargetVelocity = 650.0;
        }
        if (gamepad.getButton(GamepadKeys.Button.B) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            intakeTargetVelocity = 0.0;
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            rightShooterTargetVelocity += 100.0;
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && debounceTimer.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTimer.reset();
            rightShooterTargetVelocity -= 100.0;
        }

        double leftShooterCurrentVelocity = leftShooterMotor.getVelocity();
        double leftShooterPidOutput = leftShooterPID.calculate(leftShooterCurrentVelocity, leftShooterTargetVelocity);
        double rightShooterCurrentVelocity = rightShooterMotor.getVelocity();
        double rightShooterPidOutput = rightShooterPID.calculate(rightShooterCurrentVelocity, rightShooterTargetVelocity);
        leftShooterMotor.setPower(Range.clip(leftShooterPidOutput, 0, 1));
        rightShooterMotor.setPower(Range.clip(rightShooterPidOutput, 0, 1));

        double intakeCurrentVelocity = intakeMotor.getVelocity();
        double intakePidOutput = intakePID.calculate(intakeCurrentVelocity, intakeTargetVelocity);
        intakeMotor.setPower(Range.clip(intakePidOutput, 0, 1));

        telemetry.addData("Left Shooter Target", leftShooterTargetVelocity);
        telemetry.addData("Right Shooter Target", rightShooterTargetVelocity);
        telemetry.addData("Left Shooter Current", leftShooterCurrentVelocity);
        telemetry.addData("Right Shooter Current", rightShooterCurrentVelocity);
        telemetry.addData("Left Shooter Power", "%.2f", leftShooterMotor.getPower());
        telemetry.addData("Right Shooter Power", "%.2f", rightShooterMotor.getVelocity());
        telemetry.addLine("      ");
        telemetry.addData("Intake Target", intakeTargetVelocity);
        telemetry.addData("Intake Current", intakeCurrentVelocity);
        telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
        telemetry.update();
    }
}

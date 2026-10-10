package org.firstinspires.ftc.teamcode.TeleOp.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@TeleOp (name = "Test Dual PID", group = "Test")
public class Test_TeleOp extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private GamepadEx gamepad;
    private ElapsedTime debounceTimer = new ElapsedTime();

    private PIDController shooterPID = new PIDController(0.001, 0.0, 0.0);
    private PIDController intakePID = new PIDController(0.001, 0.0, 0.0);

    private double shooterTargetVelocity = 0.0;
    private double intakeTargetVelocity = 0.0;

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Left_Shooter_Motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop(){

        if (gamepad.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            shooterTargetVelocity = 2000.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.Y) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            shooterTargetVelocity = 0.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_UP) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            shooterTargetVelocity += 100.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_DOWN) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            shooterTargetVelocity -= 100.0;
        }

        if (gamepad.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            intakeTargetVelocity = 500.0;
        }
        if (gamepad.getButton(GamepadKeys.Button.B) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            intakeTargetVelocity = 0.0;
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            intakeTargetVelocity += 50.0;
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && debounceTimer.seconds() >= 0.2) {
            debounceTimer.reset();
            intakeTargetVelocity -= 50.0;
        }

        double shooterCurrentVelocity = shooterMotor.getVelocity();
        double shooterPidOutput = shooterPID.calculate(shooterCurrentVelocity, shooterTargetVelocity);
        shooterMotor.setPower(Range.clip(shooterPidOutput, 0, 1));

        double intakeCurrentVelocity = intakeMotor.getVelocity();
        double intakePidOutput = intakePID.calculate(intakeCurrentVelocity, intakeTargetVelocity);
        intakeMotor.setPower(Range.clip(intakePidOutput, 0, 1));

        telemetry.addData("Shooter Target", shooterTargetVelocity);
        telemetry.addData("Shooter Current", shooterCurrentVelocity);
        telemetry.addData("Shooter Power", "%.2f", shooterMotor.getPower());
        telemetry.addData("Intake Target", intakeTargetVelocity);
        telemetry.addData("Intake Current", intakeCurrentVelocity);
        telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
        telemetry.update();
    }
}

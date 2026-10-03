package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@TeleOp (name = "Test", group = "Test")
public class Test_TeleOp extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private GamepadEx gamepad;
    private ElapsedTime debounceTimer = new ElapsedTime();

    @Override
    public void init() {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter_Motor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void loop(){
        if (gamepad.getButton(GamepadKeys.Button.X) && debounceTimer.seconds() >= 0.2) {
            shooterMotor.setPower(0.7);
            debounceTimer.reset();
        }

        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER) && debounceTimer.seconds() >= 0.2) {
            shooterMotor.setPower(Math.max(-1.0, shooterMotor.getPower() - 0.01));
            debounceTimer.reset();
        }

        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER) && debounceTimer.seconds() >= 0.2) {
            shooterMotor.setPower(Math.min(1.0, shooterMotor.getPower() + 0.01));
            debounceTimer.reset();
        }

        if (gamepad.getButton(GamepadKeys.Button.A) && debounceTimer.seconds() >= 0.2) {
            intakeMotor.setPower(0.5);
            debounceTimer.reset();
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT) && debounceTimer.seconds() >= 0.2) {
            intakeMotor.setPower(Math.max(0.2, intakeMotor.getPower() - 0.01));
            debounceTimer.reset();
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)&& debounceTimer.seconds() >= 0.2) {
            intakeMotor.setPower(Math.min(1.0, intakeMotor.getPower() + 0.01));
            debounceTimer.reset();
        }
        if (gamepad.getButton(GamepadKeys.Button.B)&& debounceTimer.seconds() >= 0.2) {
            intakeMotor.setPower(0);
            shooterMotor.setPower(0);
            debounceTimer.reset();
        }
        telemetry.addData("Shooter Power","%.2f", shooterMotor.getPower());
        telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
        telemetry.update();
    }
}


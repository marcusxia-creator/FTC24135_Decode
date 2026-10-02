package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

@TeleOp (name = "Johnny Test", group = "Test")
public class Test_TeleOp extends OpMode {
    private DcMotorEx shooterMotor;
    private DcMotorEx intakeMotor;
    private GamepadEx gamepad;

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
        if (gamepad.getButton(GamepadKeys.Button.X)) {
            shooterMotor.setPower(0.5);
        }

        if (gamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            shooterMotor.setPower(
                    Math.max(-1.0, shooterMotor.getPower() - 0.01)
            );
        }

        if (gamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            shooterMotor.setPower(
                    Math.min(1.0, shooterMotor.getPower() + 0.01)
            );
        }

        if (gamepad.getButton(GamepadKeys.Button.A)) {
            intakeMotor.setPower(0.5);
        }
        if (gamepad.getButton(GamepadKeys.Button.DPAD_LEFT)) {
            intakeMotor.setPower(
                    Math.max(0.2, intakeMotor.getPower() - 0.01)
            );
        }

        if (gamepad.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            intakeMotor.setPower(
                    Math.min(1.0, intakeMotor.getPower() + 0.01)
            );
        }
        if (gamepad.getButton(GamepadKeys.Button.B)) {
            intakeMotor.setPower(0);
            shooterMotor.setPower(0);
        }
        telemetry.addData("Shooter Power","%.2f", shooterMotor.getPower());
        telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
        telemetry.update();
    }
}


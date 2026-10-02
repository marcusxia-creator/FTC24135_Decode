package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class IntakeMotorTest extends OpMode {
    DcMotorEx intakeMotor1;
    DcMotorEx intakeMotor2;

    @Override
    public void init() {
        intakeMotor1=hardwareMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor2=hardwareMap.get(DcMotorEx.class,"intakeMotor");

        intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        intakeMotor1.setPower(gamepad1.right_stick_y);
        intakeMotor2.setPower(gamepad1.right_stick_y);

        telemetry.addData("1. Power",intakeMotor1.getPower());
        telemetry.addData("1. Velocity",intakeMotor1.getVelocity());
        telemetry.addData("1. Current",intakeMotor1.getCurrent(CurrentUnit.AMPS));


        telemetry.addData("2. Power",intakeMotor2.getPower());
        telemetry.addData("2. Velocity",intakeMotor2.getVelocity());
        telemetry.addData("2. Current",intakeMotor2.getCurrent(CurrentUnit.AMPS));
    }
}

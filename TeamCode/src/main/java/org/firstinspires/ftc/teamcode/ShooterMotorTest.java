package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
public class ShooterMotorTest extends OpMode {
    DcMotorEx shooterMotor;

    @Override
    public void init() {
        shooterMotor =hardwareMap.get(DcMotorEx.class,"Shooter_Motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        shooterMotor.setPower(gamepad1.right_stick_y);
        telemetry.addData("Power", shooterMotor.getPower());
        telemetry.addData("Velocity", shooterMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Current", shooterMotor.getCurrent(CurrentUnit.AMPS));
    }
}

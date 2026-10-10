package org.firstinspires.ftc.teamcode.TeleOp;

import android.os.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class ShooterTest extends OpMode {

    private DcMotorEx leftShooterMotor;
    private DcMotorEx rightShooterMotor;

    private double shooterPower = 0.3;

    private ElapsedTime debounceTime = new ElapsedTime();
    private double DEBOUNCE_THRESHOLD = 0.25;

    @Override
    public void init() {
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "Left_Shooter_Motor");
        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "Right_Shooter_Motor");

        //leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        if (gamepad1.x && debounceTime.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTime.reset();
            leftShooterMotor.setPower(Range.clip(shooterPower, 0, 1));
            rightShooterMotor.setPower(Range.clip(shooterPower, 0, 1));
        }

        if (gamepad1.y && debounceTime.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTime.reset();
            leftShooterMotor.setPower(0.0);
            rightShooterMotor.setPower(0.0);
        }

        if (gamepad1.dpad_up && debounceTime.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTime.reset();
            shooterPower += 0.1;
        }

        if (gamepad1.dpad_down && debounceTime.seconds() >= DEBOUNCE_THRESHOLD) {
            debounceTime.reset();
            shooterPower -= 0.1;
        }

        telemetry.addData("shotoerPower", shooterPower);

    }
}

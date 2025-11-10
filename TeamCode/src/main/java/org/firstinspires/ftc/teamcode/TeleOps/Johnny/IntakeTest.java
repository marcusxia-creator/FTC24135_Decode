package org.firstinspires.ftc.teamcode.TeleOps.Johnny;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class IntakeTest extends OpMode {
    private DcMotor intakeMotor;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // Put initialization blocks here
    }

    public void loop() {
        intakeMotor.setPower(0);
        // Put loop blocks here
        if (gamepad1.right_bumper) {
            intakeMotor.setPower(1);
        }
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(-1);
        }
    }
}

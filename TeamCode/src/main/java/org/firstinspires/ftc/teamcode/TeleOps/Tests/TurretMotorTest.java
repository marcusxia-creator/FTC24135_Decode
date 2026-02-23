package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOps.Turret;

@TeleOp(name = "Turret Motor Test", group = "Test")
public class TurretMotorTest extends OpMode {

    private DcMotorEx turretMotor;

    private boolean xPrev = false;
    private boolean resetTurret = false;
    private int startingTick = 0;
    private boolean yPrev = false;
    Turret turret;

    @Override
    public void init() {
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret_Motor");

        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Turret Test Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        // =========================
        // Manual drive
        // =========================
        if (!resetTurret) {
            if (gamepad1.a) turretMotor.setPower(0.25);
            else if (gamepad1.b) turretMotor.setPower(-0.25);
            else turretMotor.setPower(0);
        }

        // =========================
        // X button: stop + reset encoder (single press)
        // =========================
        boolean xNow = gamepad1.x;
        boolean yNow = gamepad1.y;

        if (xNow && !xPrev) {
            turretMotor.setPower(0);
            turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        if (yNow && !yPrev) {
            resetTurret = true;
            startingTick = turretMotor.getCurrentPosition();  // latch once
        }

        yPrev = yNow;

        if (resetTurret) {
            if (turret.turretReset(startingTick)) {
                resetTurret = false; // done
            }
        }

        xPrev = xNow;

        // =========================
        // Telemetry
        // =========================
        telemetry.addData("Motor Power", turretMotor.getPower());
        telemetry.addData("Encoder Position", turretMotor.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        turretMotor.setPower(0);
    }
}

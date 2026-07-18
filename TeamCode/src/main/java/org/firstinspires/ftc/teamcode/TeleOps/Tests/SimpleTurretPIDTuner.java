package org.firstinspires.ftc.teamcode.TeleOps.Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TeleOps.Tests.SimpleTurretController;

@Config
@TeleOp(name = "Simple Turret PID Tuner", group = "Tuning")

public class SimpleTurretPIDTuner extends OpMode {

    /*
     * These variables can be changed from FTC Dashboard.
     */
    public static double kP = 0.003;
    public static double kI = 0.0;
    public static double kD = 0.0001;

    public static double kS = 0.05;

    public static int targetTicks = 0;

    public static int minimumTicks = -1000;
    public static int maximumTicks = 1000;

    public static int targetToleranceTicks = 10;

    private DcMotorEx turretMotor;
    private SimpleTurretController turretController;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                "turretMotor"
        );

        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        /*
         * Only reset when the turret is physically at its zero point.
         */
        turretMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        turretMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        turretController = new SimpleTurretController(
                turretMotor,
                kP,
                kI,
                kD,
                kS,
                minimumTicks,
                maximumTicks
        );

        telemetry.addLine("Simple turret tuner initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        /*
         * Update controller gains from FTC Dashboard every loop.
         */
        turretController.setPID(kP, kI, kD);
        turretController.setKS(kS);

        /*
         * Dashboard controls the target.
         */
        turretController.setTargetTicks(targetTicks);

        /*
         * Optional gamepad target controls.
         */
        if (gamepad1.dpad_left) {
            targetTicks -= 5;
        }

        if (gamepad1.dpad_right) {
            targetTicks += 5;
        }

        if (gamepad1.a) {
            targetTicks = 0;
        }

        turretController.update();

        telemetry.addData(
                "Current ticks",
                turretController.getCurrentTicks()
        );

        telemetry.addData(
                "Target ticks",
                turretController.getTargetTicks()
        );

        telemetry.addData(
                "Position error",
                "%.1f",
                turretController.getPositionError()
        );

        telemetry.addData(
                "PID output",
                "%.4f",
                turretController.getPidOutput()
        );

        telemetry.addData(
                "Feedforward output",
                "%.4f",
                turretController.getFeedforwardOutput()
        );

        telemetry.addData(
                "Motor output",
                "%.4f",
                turretController.getMotorOutput()
        );

        telemetry.addData(
                "At target",
                turretController.atTarget(targetToleranceTicks)
        );

        telemetry.addData("kP", kP);
        telemetry.addData("kI", kI);
        telemetry.addData("kD", kD);
        telemetry.addData("kS", kS);

        telemetry.update();
    }

    @Override
    public void stop() {
        turretController.stop();
    }
}

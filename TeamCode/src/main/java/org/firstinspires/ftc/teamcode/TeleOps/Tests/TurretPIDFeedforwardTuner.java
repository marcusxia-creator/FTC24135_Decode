package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TeleOps.Tests.TurretController;

@TeleOp(name = "Turret PID + FF Tuner", group = "Tuning")
public class TurretPIDFeedforwardTuner extends LinearOpMode {

    /*
     * Change these values during tuning.
     */
    private static double kP = 0.003;
    private static double kI = 0.0;
    private static double kD = 0.0001;

    /*
     * Feedforward values.
     *
     * If velocity is measured in ticks/second:
     *
     * kV is approximately:
     * maximum motor output / maximum measured velocity
     */
    private static double kS = 0.05;
    private static double kV = 0.0004;
    private static double kA = 0.00001;

    private static double maxVelocityTicksPerSecond = 1200.0;

    private static double maxAccelerationTicksPerSecondSquared =
            2400.0;

    /*
     * Replace these limits with your real mechanical limits.
     */
    private static final int MINIMUM_TURRET_TICK = -1000;
    private static final int MAXIMUM_TURRET_TICK = 1000;

    private static final int SMALL_TARGET_STEP = 10;
    private static final int LARGE_TARGET_STEP = 100;

    private static final double POSITION_TOLERANCE_TICKS = 10.0;
    private static final double VELOCITY_TOLERANCE_TICKS_PER_SECOND = 20.0;

    private DcMotorEx turretMotor;
    private TurretController turretController;

    private boolean previousDpadLeft;
    private boolean previousDpadRight;
    private boolean previousDpadUp;
    private boolean previousDpadDown;

    private boolean previousA;
    private boolean previousB;
    private boolean previousX;
    private boolean previousY;

    private boolean previousLeftBumper;
    private boolean previousRightBumper;

    private final ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        turretMotor = hardwareMap.get(
                DcMotorEx.class,
                "Turret_Motor"
        );

        turretMotor.setDirection(DcMotor.Direction.FORWARD);
        turretMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        /*
         * Only reset the encoder when the turret is physically at its
         * known zero position.
         *
         * For a competition robot, normally zero the turret using a
         * limit switch before resetting the encoder.
         */
        turretMotor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        turretMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        turretController = new TurretController(
                turretMotor,
                kP,
                kI,
                kD,
                kS,
                kV,
                kA,
                maxVelocityTicksPerSecond,
                maxAccelerationTicksPerSecondSquared,
                MINIMUM_TURRET_TICK,
                MAXIMUM_TURRET_TICK
        );

        telemetry.addLine("Turret PID + Feedforward Tuner");
        telemetry.addLine();
        telemetry.addLine("D-pad left/right: +/- 10 ticks");
        telemetry.addLine("D-pad up/down: +/- 100 ticks");
        telemetry.addLine("A: target 0");
        telemetry.addLine("X: target minimum");
        telemetry.addLine("B: target maximum");
        telemetry.addLine("Y: reset profile at current position");
        telemetry.addLine("Bumpers: manual target movement");
        telemetry.update();

        waitForStart();

        telemetryTimer.reset();

        while (opModeIsActive()) {

            updateControllerConstants();
            processTargetControls();

            turretController.update();

            displayTelemetry();
        }

        turretController.stop();
    }

    private void updateControllerConstants() {
        turretController.setPID(
                kP,
                kI,
                kD
        );

        turretController.setFeedforward(
                kS,
                kV,
                kA
        );

        turretController.setMotionLimits(
                maxVelocityTicksPerSecond,
                maxAccelerationTicksPerSecondSquared
        );
    }

    private void processTargetControls() {

        boolean dpadLeftPressed =
                gamepad1.dpad_left && !previousDpadLeft;

        boolean dpadRightPressed =
                gamepad1.dpad_right && !previousDpadRight;

        boolean dpadUpPressed =
                gamepad1.dpad_up && !previousDpadUp;

        boolean dpadDownPressed =
                gamepad1.dpad_down && !previousDpadDown;

        boolean aPressed =
                gamepad1.a && !previousA;

        boolean bPressed =
                gamepad1.b && !previousB;

        boolean xPressed =
                gamepad1.x && !previousX;

        boolean yPressed =
                gamepad1.y && !previousY;

        if (dpadLeftPressed) {
            turretController.addTargetTicks(
                    -SMALL_TARGET_STEP
            );
        }

        if (dpadRightPressed) {
            turretController.addTargetTicks(
                    SMALL_TARGET_STEP
            );
        }

        if (dpadDownPressed) {
            turretController.addTargetTicks(
                    -LARGE_TARGET_STEP
            );
        }

        if (dpadUpPressed) {
            turretController.addTargetTicks(
                    LARGE_TARGET_STEP
            );
        }

        if (aPressed) {
            turretController.setTargetPositionTicks(0);
        }

        if (xPressed) {
            turretController.setTargetPositionTicks(
                    MINIMUM_TURRET_TICK
            );
        }

        if (bPressed) {
            turretController.setTargetPositionTicks(
                    MAXIMUM_TURRET_TICK
            );
        }

        if (yPressed) {
            turretController.resetProfileToCurrentPosition();
        }

        /*
         * Continuous target movement.
         */
        double manualTargetSpeedTicksPerSecond = 400.0;

        if (gamepad1.left_bumper) {
            int tickChange = (int) (
                    -manualTargetSpeedTicksPerSecond * 0.02
            );

            turretController.addTargetTicks(tickChange);
        }

        if (gamepad1.right_bumper) {
            int tickChange = (int) (
                    manualTargetSpeedTicksPerSecond * 0.02
            );

            turretController.addTargetTicks(tickChange);
        }

        previousDpadLeft = gamepad1.dpad_left;
        previousDpadRight = gamepad1.dpad_right;
        previousDpadUp = gamepad1.dpad_up;
        previousDpadDown = gamepad1.dpad_down;

        previousA = gamepad1.a;
        previousB = gamepad1.b;
        previousX = gamepad1.x;
        previousY = gamepad1.y;

        previousLeftBumper = gamepad1.left_bumper;
        previousRightBumper = gamepad1.right_bumper;
    }

    private void displayTelemetry() {

        if (telemetryTimer.milliseconds() < 50.0) {
            return;
        }

        telemetryTimer.reset();

        int currentPosition =
                turretController.getCurrentPositionTicks();

        int targetPosition =
                turretController.getTargetPositionTicks();

        double commandedPosition =
                turretController.getCommandedPositionTicks();

        double commandedVelocity =
                turretController
                        .getCommandedVelocityTicksPerSecond();

        double measuredVelocity =
                turretController
                        .getMeasuredVelocityTicksPerSecond();

        telemetry.addData(
                "Current position",
                currentPosition
        );

        telemetry.addData(
                "Profile position",
                "%.1f",
                commandedPosition
        );

        telemetry.addData(
                "Final target",
                targetPosition
        );

        telemetry.addData(
                "Profile error",
                "%.1f",
                turretController.getPositionErrorTicks()
        );

        telemetry.addData(
                "Final target error",
                "%.1f",
                turretController.getFinalTargetErrorTicks()
        );

        telemetry.addData(
                "Commanded velocity",
                "%.1f ticks/s",
                commandedVelocity
        );

        telemetry.addData(
                "Measured velocity",
                "%.1f ticks/s",
                measuredVelocity
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
                turretController.atTarget(
                        POSITION_TOLERANCE_TICKS,
                        VELOCITY_TOLERANCE_TICKS_PER_SECOND
                )
        );

        telemetry.addLine();
        telemetry.addData("kP", "%.7f", kP);
        telemetry.addData("kI", "%.7f", kI);
        telemetry.addData("kD", "%.7f", kD);
        telemetry.addData("kS", "%.7f", kS);
        telemetry.addData("kV", "%.7f", kV);
        telemetry.addData("kA", "%.7f", kA);

        telemetry.update();
    }
}

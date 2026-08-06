package org.firstinspires.ftc.teamcode.TeleOps.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.DriveMotionController;

/**
 * Test TeleOp for DriveMotionController.
 *
 * Controls:
 *
 * Left stick Y:
 *      Manual forward/backward
 *
 * Left stick X:
 *      Manual strafe
 *
 * Right stick X:
 *      Manual rotation
 *
 * A:
 *      Start semi-auto straight-line movement
 *
 * B:
 *      Cancel semi-auto immediately
 *
 * X:
 *      Reset Pinpoint pose to 0, 0, 0
 *
 * Any joystick movement above the override threshold:
 *      Cancel semi-auto and return to manual control
 */
@Config
@TeleOp(
        name = "Motion Drive Test",
        group = "org.firstinspires.ftc.teamcode.Tests"
)
public class MotionDriveTeleOp extends OpMode {

    private RobotHardware robot;

    private final DriveMotionController driveMotionController =
            new DriveMotionController();

    /*
     * ============================================================
     * Dashboard-configurable target
     * ============================================================
     */

    public static double targetXMM = 600.0;
    public static double targetYMM = 0.0;
    public static double targetHeadingDegrees = 0.0;

    /*
     * Manual drive settings.
     */
    public static double manualDriveScale = 0.70;
    public static double manualTurnScale = 0.60;

    /*
     * Any joystick input greater than this cancels semi-auto.
     */
    public static double manualOverrideThreshold = 0.15;

    /*
     * Set true if you want A to create a target relative to the
     * robot's current field position.
     *
     * false:
     *      targetXMM and targetYMM are absolute field coordinates.
     *
     * true:
     *      targetXMM and targetYMM are offsets from the current pose.
     */
    public static boolean useRelativeTarget = true;

    /*
     * ============================================================
     * Button edge tracking
     * ============================================================
     */

    private boolean previousAButton = false;
    private boolean previousBButton = false;
    private boolean previousXButton = false;

    /*
     * Cached pose. Pinpoint is updated exactly once per loop.
     */
    private Pose2D currentPose;

    /*
     * Status message for telemetry.
     */
    private String lastAction = "Waiting";

    /*
     * Loop-frequency measurement.
     */
    private long previousLoopTimeNs = 0L;
    private double loopFrequencyHz = 0.0;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        robot = new RobotHardware(hardwareMap);

        /*
         * These calls match your existing TeleOp/TestTeleOp pattern.
         */
        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        /*
         * Use this only if the method exists in your RobotHardware.
         * Your competition TeleOp uses it.
         */
        robot.initializeBulkReading(hardwareMap);

        stopDriveMotors();

        telemetry.addLine("Motion Drive Test initialized");
        telemetry.addLine("A: start semi-auto");
        telemetry.addLine("B: cancel");
        telemetry.addLine("X: reset Pinpoint pose");
        telemetry.addLine("Joysticks: manual drive / override");
        telemetry.update();
    }

    @Override
    public void start() {

        previousLoopTimeNs = System.nanoTime();

        /*
         * Begin with all drivetrain outputs at zero.
         */
        stopDriveMotors();

        lastAction = "TeleOp started";
    }

    @Override
    public void loop() {

        /*
         * ========================================================
         * 1. Start hardware loop
         * ========================================================
         *
         * Your competition TeleOp clears bulk cache once at the
         * beginning of every loop.
         */
        robot.clearBulkCache();

        updateLoopFrequency();

        /*
         * ========================================================
         * 2. Update Pinpoint exactly once
         * ========================================================
         */
        robot.pinpoint.update();

        currentPose =
                robot.pinpoint.getPosition();

        /*
         * ========================================================
         * 3. Read button edges
         * ========================================================
         */

        boolean aPressed =
                gamepad1.a && !previousAButton;

        boolean bPressed =
                gamepad1.b && !previousBButton;

        boolean xPressed =
                gamepad1.x && !previousXButton;

        previousAButton = gamepad1.a;
        previousBButton = gamepad1.b;
        previousXButton = gamepad1.x;

        /*
         * ========================================================
         * 4. Reset pose
         * ========================================================
         */

        if (xPressed) {

            if (driveMotionController.isActive()) {
                driveMotionController.cancel();
            }

            Pose2D resetPose =
                    new Pose2D(
                            DistanceUnit.MM,
                            0.0,
                            0.0,
                            AngleUnit.DEGREES,
                            0.0
                    );

            robot.pinpoint.setPosition(resetPose);

            currentPose = resetPose;

            stopDriveMotors();

            lastAction = "Pinpoint reset";
        }

        /*
         * ========================================================
         * 5. Start semi-auto
         * ========================================================
         */

        if (aPressed) {

            Pose2D targetPose =
                    createTargetPose(currentPose);

            driveMotionController.startStraightLine(
                    currentPose,
                    targetPose
            );

            lastAction =
                    String.format(
                            "Semi-auto started: X %.0f, Y %.0f, H %.1f",
                            targetPose.getX(DistanceUnit.MM),
                            targetPose.getY(DistanceUnit.MM),
                            targetPose.getHeading(AngleUnit.DEGREES)
                    );
        }

        /*
         * ========================================================
         * 6. Explicit cancel
         * ========================================================
         */

        if (bPressed
                && driveMotionController.isActive()) {

            driveMotionController.cancel();

            stopDriveMotors();

            lastAction = "Semi-auto cancelled with B";
        }

        /*
         * ========================================================
         * 7. Driver joystick override
         * ========================================================
         */

        boolean driverOverride =
                hasManualOverrideInput();

        if (driverOverride
                && driveMotionController.isActive()) {

            driveMotionController.cancel();

            stopDriveMotors();

            lastAction = "Semi-auto cancelled by joystick";
        }

        /*
         * ========================================================
         * 8. Select drivetrain owner
         * ========================================================
         *
         * Only one part of the program should write drivetrain
         * powers during a loop.
         */

        if (driveMotionController.isActive()) {

            runSemiAutoDrive();

        } else {

            runManualDrive();
        }

        /*
         * ========================================================
         * 9. Completion status
         * ========================================================
         */

        if (driveMotionController.isFinishedSuccessfully()) {

            lastAction = "Semi-auto completed";

        } else if (driveMotionController.hasTimedOut()) {

            lastAction = "Semi-auto timed out";

        } else if (driveMotionController.wasCancelled()) {

            /*
             * Preserve a more specific cancellation message if one
             * was already set above.
             */
            if (!lastAction.contains("cancelled")) {
                lastAction = "Semi-auto cancelled";
            }
        }

        /*
         * ========================================================
         * 10. Telemetry
         * ========================================================
         */

        sendTelemetry();
    }

    @Override
    public void stop() {

        driveMotionController.cancel();

        stopDriveMotors();
    }

    /**
     * Builds either an absolute or relative target pose.
     */
    private Pose2D createTargetPose(
            Pose2D startPose
    ) {

        double targetX;
        double targetY;

        if (useRelativeTarget) {

            targetX =
                    startPose.getX(DistanceUnit.MM)
                            + targetXMM;

            targetY =
                    startPose.getY(DistanceUnit.MM)
                            + targetYMM;

        } else {

            targetX = targetXMM;
            targetY = targetYMM;
        }

        return new Pose2D(
                DistanceUnit.MM,
                targetX,
                targetY,
                AngleUnit.DEGREES,
                targetHeadingDegrees
        );
    }

    /**
     * Runs one semi-auto controller update and applies its calculated
     * mecanum motor powers.
     */
    private void runSemiAutoDrive() {

        driveMotionController.update(
                currentPose
        );

        applyDriveMotorPowers(
                driveMotionController
                        .getLeftFrontPower(),

                driveMotionController
                        .getRightFrontPower(),

                driveMotionController
                        .getLeftBackPower(),

                driveMotionController
                        .getRightBackPower()
        );
    }

    /**
     * Manual robot-centric mecanum drive.
     *
     * This is intentionally kept independent of RobotDrive so the
     * test isolates DriveMotionController.
     */
    private void runManualDrive() {

        double forward =
                -gamepad1.left_stick_y
                        * manualDriveScale;

        double strafe =
                gamepad1.left_stick_x
                        * manualDriveScale;

        double turn =
                gamepad1.right_stick_x
                        * manualTurnScale;

        /*
         * Optional joystick deadband.
         */
        forward = applyDeadband(forward, 0.05);
        strafe = applyDeadband(strafe, 0.05);
        turn = applyDeadband(turn, 0.05);

        double leftFront =
                forward + strafe + turn;

        double rightFront =
                forward - strafe - turn;

        double leftBack =
                forward - strafe + turn;

        double rightBack =
                forward + strafe - turn;

        double maximumMagnitude =
                Math.max(
                        1.0,
                        Math.max(
                                Math.abs(leftFront),
                                Math.max(
                                        Math.abs(rightFront),
                                        Math.max(
                                                Math.abs(leftBack),
                                                Math.abs(rightBack)
                                        )
                                )
                        )
                );

        applyDriveMotorPowers(
                leftFront / maximumMagnitude,
                rightFront / maximumMagnitude,
                leftBack / maximumMagnitude,
                rightBack / maximumMagnitude
        );
    }

    /**
     * Returns true when the driver moves any drive-control joystick
     * past the override threshold.
     */
    private boolean hasManualOverrideInput() {

        return Math.abs(gamepad1.left_stick_x)
                > manualOverrideThreshold

                || Math.abs(gamepad1.left_stick_y)
                > manualOverrideThreshold

                || Math.abs(gamepad1.right_stick_x)
                > manualOverrideThreshold;
    }

    /**
     * Applies powers using the drivetrain motors already configured
     * and direction-corrected by RobotHardware.init().
     *
     * Do not reverse motors again in this test class.
     */
    private void applyDriveMotorPowers(
            double leftFront,
            double rightFront,
            double leftBack,
            double rightBack
    ) {

        robot.frontLeftMotor.setPower(
                Range.clip(leftFront, -1.0, 1.0)
        );

        robot.frontRightMotor.setPower(
                Range.clip(rightFront, -1.0, 1.0)
        );

        robot.backLeftMotor.setPower(
                Range.clip(leftBack, -1.0, 1.0)
        );

        robot.backRightMotor.setPower(
                Range.clip(rightBack, -1.0, 1.0)
        );
    }

    private void stopDriveMotors() {

        if (robot == null) {
            return;
        }

        applyDriveMotorPowers(
                0.0,
                0.0,
                0.0,
                0.0
        );
    }

    private void sendTelemetry() {

        telemetry.addData(
                "Mode",
                driveMotionController.isActive()
                        ? "SEMI-AUTO"
                        : "MANUAL"
        );

        telemetry.addData(
                "Last action",
                lastAction
        );

        telemetry.addData(
                "Loop frequency",
                "%.1f Hz",
                loopFrequencyHz
        );

        telemetry.addLine();

        telemetry.addData(
                "Current X",
                "%.1f mm",
                currentPose.getX(DistanceUnit.MM)
        );

        telemetry.addData(
                "Current Y",
                "%.1f mm",
                currentPose.getY(DistanceUnit.MM)
        );

        telemetry.addData(
                "Current heading",
                "%.2f deg",
                currentPose.getHeading(AngleUnit.DEGREES)
        );

        telemetry.addLine();

        telemetry.addData(
                "Planned X",
                "%.1f mm",
                driveMotionController.getPlannedXMM()
        );

        telemetry.addData(
                "Planned Y",
                "%.1f mm",
                driveMotionController.getPlannedYMM()
        );

        telemetry.addData(
                "Planned distance",
                "%.1f mm",
                driveMotionController.getPlannedDistanceMM()
        );

        telemetry.addData(
                "Planned velocity",
                "%.1f mm/s",
                driveMotionController
                        .getPlannedVelocityMMPerSecond()
        );

        telemetry.addData(
                "Planned acceleration",
                "%.1f mm/s²",
                driveMotionController
                        .getPlannedAccelerationMMPerSecondSquared()
        );

        telemetry.addLine();

        telemetry.addData(
                "X tracking error",
                "%.1f mm",
                driveMotionController.getXTrackingErrorMM()
        );

        telemetry.addData(
                "Y tracking error",
                "%.1f mm",
                driveMotionController.getYTrackingErrorMM()
        );

        telemetry.addData(
                "Final position error",
                "%.1f mm",
                driveMotionController.getFinalPositionErrorMM()
        );

        telemetry.addData(
                "Final heading error",
                "%.2f deg",
                Math.toDegrees(
                        driveMotionController
                                .getFinalHeadingErrorRadians()
                )
        );

        telemetry.addLine();

        telemetry.addData(
                "X feedforward",
                "%.4f",
                driveMotionController.getXFeedforwardOutput()
        );

        telemetry.addData(
                "Y feedforward",
                "%.4f",
                driveMotionController.getYFeedforwardOutput()
        );

        telemetry.addData(
                "X PID",
                "%.4f",
                driveMotionController.getXPIDOutput()
        );

        telemetry.addData(
                "Y PID",
                "%.4f",
                driveMotionController.getYPIDOutput()
        );

        telemetry.addData(
                "Heading PID",
                "%.4f",
                driveMotionController.getHeadingPIDOutput()
        );

        telemetry.addLine();

        telemetry.addData(
                "Left front power",
                "%.3f",
                driveMotionController.getLeftFrontPower()
        );

        telemetry.addData(
                "Right front power",
                "%.3f",
                driveMotionController.getRightFrontPower()
        );

        telemetry.addData(
                "Left back power",
                "%.3f",
                driveMotionController.getLeftBackPower()
        );

        telemetry.addData(
                "Right back power",
                "%.3f",
                driveMotionController.getRightBackPower()
        );

        telemetry.addLine();

        telemetry.addData(
                "Profile time",
                "%.3f / %.3f s",
                driveMotionController
                        .getProfileElapsedTimeSeconds(),
                driveMotionController
                        .getProfileTotalTimeSeconds()
        );

        telemetry.addData(
                "Settling",
                driveMotionController.isSettling()
        );

        telemetry.addData(
                "Completed",
                driveMotionController
                        .isFinishedSuccessfully()
        );

        telemetry.addData(
                "Cancelled",
                driveMotionController.wasCancelled()
        );

        telemetry.addData(
                "Timed out",
                driveMotionController.hasTimedOut()
        );

        telemetry.update();
    }

    private void updateLoopFrequency() {

        long currentTimeNs =
                System.nanoTime();

        if (previousLoopTimeNs != 0L) {

            double deltaTimeSeconds =
                    (
                            currentTimeNs
                                    - previousLoopTimeNs
                    ) / 1_000_000_000.0;

            if (deltaTimeSeconds > 0.0) {
                loopFrequencyHz =
                        1.0 / deltaTimeSeconds;
            }
        }

        previousLoopTimeNs =
                currentTimeNs;
    }

    private static double applyDeadband(
            double value,
            double deadband
    ) {

        if (Math.abs(value) < deadband) {
            return 0.0;
        }

        return value;
    }
}
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
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.DriveMotionController;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.RobotVelocity;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory.DriveTrajectory;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory.TrajectoryPlanner;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory.TrajectorySegment;

import java.util.List;

/**
 * Test TeleOp for the trajectory-build pipeline: TrajectoryBuilder,
 * TrajectoryPlanner, TrajectoryProfile, DriveTrajectory, and
 * DriveMotionController.startTrajectory(...).
 *
 * This is a separate OpMode from "Motion Drive Test" on purpose: that
 * one already has a working, tuned single-straight-line LQR workflow,
 * and this class lets the multi-segment trajectory system be debugged
 * without touching it.
 *
 * Builds a fixed 2-segment trajectory on button press:
 *
 * start pose --lineTo--> waypoint 1 --splineTo--> waypoint 2
 *
 * All waypoint coordinates and cornering/velocity tuning values are
 * FTC Dashboard configurable, same as Motion Drive Test.
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
 *      Build and start following the trajectory
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
        name = "Trajectory Drive Test",
        group = "org.firstinspires.ftc.teamcode.Tests"
)
public class TrajectoryDriveTeleOp extends OpMode {

    private RobotHardware robot;

    private final DriveMotionController driveMotionController =
            new DriveMotionController();

    /*
     * The trajectory currently being followed, kept around only for
     * telemetry (total distance/time/segment count).
     */
    private DriveTrajectory activeTrajectory;

    /*
     * ============================================================
     * Dashboard-configurable waypoints
     * ============================================================
     *
     * useRelativeWaypoints = true:
     *      waypoint coordinates are offsets from the pose the robot
     *      is at when A is pressed.
     *
     * useRelativeWaypoints = false:
     *      waypoint coordinates are absolute field coordinates.
     */

    public static boolean useRelativeWaypoints = true;

    public static double waypoint1XMM = 600.0;
    public static double waypoint1YMM = 0.0;

    public static double waypoint2XMM = 1200.0;
    public static double waypoint2YMM = 400.0;
    public static double waypoint2HeadingDegrees = 90.0;

    public static double finalEndVelocityMMPerSecond = 0.0;

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

        robot.init();
        robot.initIMU();
        robot.initPinpoint();

        robot.initializeBulkReading(hardwareMap);

        stopDriveMotors();

        telemetry.addLine("Trajectory Drive Test initialized");
        telemetry.addLine("A: build + start trajectory");
        telemetry.addLine("B: cancel");
        telemetry.addLine("X: reset Pinpoint pose");
        telemetry.addLine("Joysticks: manual drive / override");
        telemetry.update();
    }

    @Override
    public void start() {

        previousLoopTimeNs = System.nanoTime();

        stopDriveMotors();

        lastAction = "TeleOp started";
    }

    @Override
    public void loop() {

        /*
         * ========================================================
         * 1. Start hardware loop
         * ========================================================
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
         * 5. Build + start trajectory
         * ========================================================
         */

        if (aPressed) {

            try {

                activeTrajectory = buildTrajectory(currentPose);

                driveMotionController.startTrajectory(
                        activeTrajectory
                );

                lastAction =
                        String.format(
                                "Trajectory started: %.0f mm, %.2f s, %d segments",
                                activeTrajectory.getTotalDistanceMM(),
                                activeTrajectory.getTotalTimeSeconds(),
                                activeTrajectory.getSegments().size()
                        );

            } catch (IllegalArgumentException exception) {

                /*
                 * Clear any previous trajectory so telemetry doesn't
                 * keep showing a stale "success" summary for a build
                 * that just failed.
                 */
                activeTrajectory = null;

                lastAction =
                        "Trajectory build failed: "
                                + exception.getMessage();
            }
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

            lastAction = "Trajectory cancelled with B";
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

            lastAction = "Trajectory cancelled by joystick";
        }

        /*
         * ========================================================
         * 8. Select drivetrain owner
         * ========================================================
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

            lastAction = "Trajectory completed";

        } else if (driveMotionController.hasTimedOut()) {

            lastAction = "Trajectory timed out";

        } else if (driveMotionController.wasCancelled()) {

            if (!lastAction.contains("cancelled")) {
                lastAction = "Trajectory cancelled";
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
     * Builds a fixed 2-segment trajectory: a straight line to
     * waypoint 1, then a spline into waypoint 2 at the configured
     * heading.
     */
    private DriveTrajectory buildTrajectory(
            Pose2D startPose
    ) {

        double startXMM = startPose.getX(DistanceUnit.MM);
        double startYMM = startPose.getY(DistanceUnit.MM);

        double waypoint1AbsXMM;
        double waypoint1AbsYMM;
        double waypoint2AbsXMM;
        double waypoint2AbsYMM;

        if (useRelativeWaypoints) {

            waypoint1AbsXMM = startXMM + waypoint1XMM;
            waypoint1AbsYMM = startYMM + waypoint1YMM;

            waypoint2AbsXMM = startXMM + waypoint2XMM;
            waypoint2AbsYMM = startYMM + waypoint2YMM;

        } else {

            waypoint1AbsXMM = waypoint1XMM;
            waypoint1AbsYMM = waypoint1YMM;

            waypoint2AbsXMM = waypoint2XMM;
            waypoint2AbsYMM = waypoint2YMM;
        }

        return TrajectoryBuilder.startAt(startPose)
                .lineTo(waypoint1AbsXMM, waypoint1AbsYMM)
                .splineTo(
                        waypoint2AbsXMM,
                        waypoint2AbsYMM,
                        Math.toRadians(waypoint2HeadingDegrees)
                )
                .endWithVelocity(finalEndVelocityMMPerSecond)
                .build();
    }

    /**
     * Runs one semi-auto controller update and applies its calculated
     * mecanum motor powers.
     */
    private void runSemiAutoDrive() {

        RobotVelocity currentVelocity =
                new RobotVelocity(
                        robot.pinpoint.getVelX(DistanceUnit.MM),
                        robot.pinpoint.getVelY(DistanceUnit.MM),
                        robot.pinpoint.getHeadingVelocity(
                                UnnormalizedAngleUnit.RADIANS
                        )
                );

        driveMotionController.update(
                currentPose,
                currentVelocity
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
                "Max lateral accel (cornering)",
                "%.0f mm/s²",
                TrajectoryPlanner.maximumLateralAccelerationMMPerSecondSquared
        );

        telemetry.addData(
                "Curvature sample count",
                TrajectoryPlanner.curvatureSampleCount
        );

        telemetry.addLine();

        if (activeTrajectory != null) {

            telemetry.addData(
                    "Trajectory total distance",
                    "%.1f mm",
                    activeTrajectory.getTotalDistanceMM()
            );

            telemetry.addData(
                    "Trajectory total time",
                    "%.2f s",
                    activeTrajectory.getTotalTimeSeconds()
            );

            addSegmentTelemetry();

            telemetry.addLine();
        }

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
                "X LQR",
                "%.4f",
                driveMotionController.getXPIDOutput()
        );

        telemetry.addData(
                "Y LQR",
                "%.4f",
                driveMotionController.getYPIDOutput()
        );

        telemetry.addData(
                "Heading LQR",
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

    /**
     * Prints each segment's assigned boundary velocities and duration,
     * plus which segment is currently active -- this is the actual
     * signal you need to tune TrajectoryPlanner's cornering behavior.
     * Without it you can only see whether the whole run "looked ok,"
     * not which corner slowed the robot or by how much.
     */
    private void addSegmentTelemetry() {

        double elapsedTimeSeconds =
                driveMotionController.getProfileElapsedTimeSeconds();

        List<TrajectorySegment> segments =
                activeTrajectory.getSegments();

        for (int index = 0; index < segments.size(); index++) {

            TrajectorySegment segment = segments.get(index);

            boolean isActiveSegment =
                    driveMotionController.isActive()
                            && elapsedTimeSeconds
                            >= segment.getStartTimeSeconds()
                            && elapsedTimeSeconds
                            < segment.getEndTimeSeconds();

            telemetry.addData(
                    (isActiveSegment ? "-> Seg " : "   Seg ")
                            + index,
                    "len %.0f mm | v0 %.0f -> v1 %.0f mm/s | %.2f s",
                    segment.getLengthMM(),
                    segment.getStartVelocityMMPerSecond(),
                    segment.getEndVelocityMMPerSecond(),
                    segment.getEndTimeSeconds()
                            - segment.getStartTimeSeconds()
            );
        }
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

package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.DriveMotionController;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.CubicBezierPath;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.DrivePath;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.StraightLinePath;

import java.util.ArrayList;
import java.util.List;

/**
 * Fluent API for assembling a complete DriveTrajectory out of straight
 * and curved segments.
 *
 * Example:
 *
 * DriveTrajectory trajectory =
 *      TrajectoryBuilder.startAt(startPose)
 *              .lineTo(600, 0)
 *              .splineTo(1200, 400, Math.toRadians(90))
 *              .endWithVelocity(0)
 *              .build();
 *
 * TrajectoryBuilder keeps a running "pen position" (the pose at the end
 * of the last segment added so far) and a running tangent heading, used
 * so a following splineTo(...) blends smoothly out of the previous
 * segment's direction of travel.
 *
 * build() assembles the segments into a CompositeDrivePath, hands the
 * segment list to TrajectoryPlanner to assign boundary velocities and
 * per-segment MotionProfiles, wraps the result in a TrajectoryProfile,
 * and returns the finished DriveTrajectory.
 */
public final class TrajectoryBuilder {

    /*
     * Default fraction of the straight-line chord distance used as the
     * Bezier handle length in splineTo(...). A common, simple default
     * for producing smooth, non-kinked splines -- not a unique choice.
     * If a trajectory needs tighter/looser curve control, the natural
     * next step is a splineTo(x, y, heading, handleLengthMM) overload.
     */
    private static final double DEFAULT_HANDLE_LENGTH_FRACTION = 1.0 / 3.0;

    private Pose2D currentPose;

    private final List<Pose2D> waypoints = new ArrayList<>();
    private final List<TrajectorySegment> segments = new ArrayList<>();

    private double maximumVelocityMMPerSecond =
            DriveMotionController.maximumVelocityMMPerSecond;

    private double maximumAccelerationMMPerSecondSquared =
            DriveMotionController.maximumAccelerationMMPerSecondSquared;

    private double maximumDecelerationMMPerSecondSquared =
            DriveMotionController.maximumDecelerationMMPerSecondSquared;

    private double finalEndVelocityMMPerSecond = 0.0;

    private TrajectoryBuilder(Pose2D startPose) {
        this.currentPose = startPose;
        waypoints.add(startPose);
    }

    /**
     * Begins a new trajectory at the given pose.
     */
    public static TrajectoryBuilder startAt(Pose2D startPose) {
        if (startPose == null) {
            throw new IllegalArgumentException(
                    "Start pose cannot be null."
            );
        }

        return new TrajectoryBuilder(startPose);
    }

    /**
     * Overrides the velocity/acceleration/deceleration limits used for
     * segments added after this call.
     */
    public TrajectoryBuilder withConstraints(
            double maximumVelocityMMPerSecond,
            double maximumAccelerationMMPerSecondSquared,
            double maximumDecelerationMMPerSecondSquared
    ) {
        this.maximumVelocityMMPerSecond = maximumVelocityMMPerSecond;
        this.maximumAccelerationMMPerSecondSquared =
                maximumAccelerationMMPerSecondSquared;
        this.maximumDecelerationMMPerSecondSquared =
                maximumDecelerationMMPerSecondSquared;

        return this;
    }

    /**
     * Adds a straight-line segment from the current pen position to
     * (xMM, yMM). The running tangent heading becomes this segment's
     * direction of travel, so a following splineTo(...) blends
     * smoothly out of it.
     */
    public TrajectoryBuilder lineTo(double xMM, double yMM) {
        double startXMM = currentPose.getX(DistanceUnit.MM);
        double startYMM = currentPose.getY(DistanceUnit.MM);

        StraightLinePath path = new StraightLinePath(
                startXMM,
                startYMM,
                xMM,
                yMM
        );

        addSegment(path);

        double headingRadians = Math.atan2(
                yMM - startYMM,
                xMM - startXMM
        );

        advancePenPosition(xMM, yMM, headingRadians);

        return this;
    }

    /**
     * Adds a curved segment from the current pen position/tangent
     * heading to (xMM, yMM), arriving with the given tangent heading.
     *
     * Interior Bezier control points are placed along the start and
     * end tangent directions using the default handle-length fraction
     * of the straight-line distance between the two points.
     */
    public TrajectoryBuilder splineTo(
            double xMM,
            double yMM,
            double endTangentHeadingRadians
    ) {
        double startXMM = currentPose.getX(DistanceUnit.MM);
        double startYMM = currentPose.getY(DistanceUnit.MM);
        double startTangentHeadingRadians =
                currentPose.getHeading(AngleUnit.RADIANS);

        double chordLengthMM = Math.hypot(
                xMM - startXMM,
                yMM - startYMM
        );

        double handleLengthMM =
                chordLengthMM * DEFAULT_HANDLE_LENGTH_FRACTION;

        double controlX1MM =
                startXMM + handleLengthMM * Math.cos(startTangentHeadingRadians);
        double controlY1MM =
                startYMM + handleLengthMM * Math.sin(startTangentHeadingRadians);

        double controlX2MM =
                xMM - handleLengthMM * Math.cos(endTangentHeadingRadians);
        double controlY2MM =
                yMM - handleLengthMM * Math.sin(endTangentHeadingRadians);

        CubicBezierPath path = new CubicBezierPath(
                startXMM, startYMM,
                controlX1MM, controlY1MM,
                controlX2MM, controlY2MM,
                xMM, yMM
        );

        addSegment(path);

        advancePenPosition(xMM, yMM, endTangentHeadingRadians);

        return this;
    }

    /**
     * Sets the velocity the robot should have when it reaches the end
     * of the very last segment. Defaults to 0 (full stop).
     */
    public TrajectoryBuilder endWithVelocity(
            double finalEndVelocityMMPerSecond
    ) {
        this.finalEndVelocityMMPerSecond = finalEndVelocityMMPerSecond;
        return this;
    }

    /**
     * Assembles the finished DriveTrajectory: builds the
     * CompositeDrivePath, runs TrajectoryPlanner to assign boundary
     * velocities and per-segment MotionProfiles, and wraps everything
     * in a TrajectoryProfile.
     */
    public DriveTrajectory build() {
        if (segments.isEmpty()) {
            throw new IllegalArgumentException(
                    "TrajectoryBuilder requires at least one segment "
                            + "(call lineTo(...) or splineTo(...) "
                            + "before build())."
            );
        }

        List<DrivePath> paths = new ArrayList<>();

        for (TrajectorySegment segment : segments) {
            paths.add(segment.getDrivePath());
        }

        CompositeDrivePath compositeDrivePath =
                new CompositeDrivePath(paths);

        TrajectoryPlanner.plan(segments, finalEndVelocityMMPerSecond);

        TrajectoryProfile trajectoryProfile =
                new TrajectoryProfile(segments);

        return new DriveTrajectory(
                compositeDrivePath,
                trajectoryProfile,
                currentPose,
                waypoints,
                segments
        );
    }

    private void addSegment(DrivePath path) {
        segments.add(
                new TrajectorySegment(
                        path,
                        maximumVelocityMMPerSecond,
                        maximumAccelerationMMPerSecondSquared,
                        maximumDecelerationMMPerSecondSquared
                )
        );
    }

    private void advancePenPosition(
            double xMM,
            double yMM,
            double headingRadians
    ) {
        currentPose = new Pose2D(
                DistanceUnit.MM,
                xMM,
                yMM,
                AngleUnit.RADIANS,
                headingRadians
        );

        waypoints.add(currentPose);
    }
}

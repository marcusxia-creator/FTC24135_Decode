package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The complete, ready-to-follow result of building a trajectory with
 * TrajectoryBuilder.
 *
 * DriveTrajectory bundles everything DriveMotionController needs to
 * follow a multi-segment path:
 *
 * - a CompositeDrivePath   (where to travel -- geometry)
 * - a TrajectoryProfile    (where the robot should be at time t)
 * - the final target pose  (used for heading hold + completion checks)
 * - total distance/time    (convenience, same numbers the profile has)
 * - the waypoint list       (for telemetry/field-view drawing)
 * - the segment list        (each segment's constraints, for telemetry)
 *
 * This class is immutable: once built, a DriveTrajectory cannot change.
 */
public final class DriveTrajectory {

    private final CompositeDrivePath compositeDrivePath;
    private final TrajectoryProfile trajectoryProfile;

    private final Pose2D finalTargetPose;

    private final double totalDistanceMM;
    private final double totalTimeSeconds;

    private final List<Pose2D> waypoints;
    private final List<TrajectorySegment> segments;

    public DriveTrajectory(
            CompositeDrivePath compositeDrivePath,
            TrajectoryProfile trajectoryProfile,
            Pose2D finalTargetPose,
            List<Pose2D> waypoints,
            List<TrajectorySegment> segments
    ) {
        if (compositeDrivePath == null) {
            throw new IllegalArgumentException(
                    "CompositeDrivePath cannot be null."
            );
        }

        if (trajectoryProfile == null) {
            throw new IllegalArgumentException(
                    "TrajectoryProfile cannot be null."
            );
        }

        if (finalTargetPose == null) {
            throw new IllegalArgumentException(
                    "Final target pose cannot be null."
            );
        }

        if (waypoints == null || waypoints.isEmpty()) {
            throw new IllegalArgumentException(
                    "Waypoint list cannot be null or empty."
            );
        }

        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException(
                    "Segment list cannot be null or empty."
            );
        }

        this.compositeDrivePath = compositeDrivePath;
        this.trajectoryProfile = trajectoryProfile;
        this.finalTargetPose = finalTargetPose;

        this.totalDistanceMM = compositeDrivePath.getLengthMM();
        this.totalTimeSeconds = trajectoryProfile.getTotalTimeSeconds();

        this.waypoints =
                Collections.unmodifiableList(new ArrayList<>(waypoints));

        this.segments =
                Collections.unmodifiableList(new ArrayList<>(segments));
    }

    public CompositeDrivePath getCompositeDrivePath() {
        return compositeDrivePath;
    }

    public TrajectoryProfile getTrajectoryProfile() {
        return trajectoryProfile;
    }

    public Pose2D getFinalTargetPose() {
        return finalTargetPose;
    }

    public double getTotalDistanceMM() {
        return totalDistanceMM;
    }

    public double getTotalTimeSeconds() {
        return totalTimeSeconds;
    }

    public List<Pose2D> getWaypoints() {
        return waypoints;
    }

    public List<TrajectorySegment> getSegments() {
        return segments;
    }
}

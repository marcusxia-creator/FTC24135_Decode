package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory;

import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile.MotionProfile;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile.MotionState;

import java.util.ArrayList;
import java.util.List;

/**
 * Stitches a list of TrajectorySegments (each already carrying its own
 * TrapezoidalProfile, assigned by TrajectoryPlanner) into one
 * continuous, time-indexed MotionProfile covering the whole trajectory.
 *
 * TrajectoryPlanner.plan(...) must have already run on the segment list
 * before this is constructed -- every segment must have its
 * MotionProfile, startTimeSeconds, and startDistanceAlongTrajectoryMM
 * already assigned.
 *
 * Units:
 * - distance: millimetres
 * - velocity: millimetres per second
 * - acceleration: millimetres per second squared
 * - time: seconds
 */
public final class TrajectoryProfile implements MotionProfile {

    private final List<TrajectorySegment> segments;

    private final double totalDistanceMM;
    private final double totalTimeSeconds;

    public TrajectoryProfile(List<TrajectorySegment> segments) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException(
                    "TrajectoryProfile requires at least one segment."
            );
        }

        for (TrajectorySegment segment : segments) {
            if (!segment.hasMotionProfile()) {
                throw new IllegalArgumentException(
                        "Every segment must have a MotionProfile "
                                + "assigned (run TrajectoryPlanner.plan(...) "
                                + "first)."
                );
            }
        }

        this.segments = new ArrayList<>(segments);

        TrajectorySegment lastSegment =
                this.segments.get(this.segments.size() - 1);

        this.totalDistanceMM =
                lastSegment.getEndDistanceAlongTrajectoryMM();

        this.totalTimeSeconds =
                lastSegment.getEndTimeSeconds();
    }

    @Override
    public MotionState getMotionState(double elapsedTimeSeconds) {
        double validTimeSeconds = Math.max(0.0, elapsedTimeSeconds);

        TrajectorySegment activeSegment =
                findSegmentForTime(validTimeSeconds);

        double localTimeSeconds =
                validTimeSeconds - activeSegment.getStartTimeSeconds();

        MotionState localState =
                activeSegment.getMotionProfile()
                        .getMotionState(localTimeSeconds);

        double globalDistanceMM =
                activeSegment.getStartDistanceAlongTrajectoryMM()
                        + localState.getDistanceAlongPathMM();

        double globalProgress = totalDistanceMM > 0.0
                ? globalDistanceMM / totalDistanceMM
                : 1.0;

        return new MotionState(
                validTimeSeconds,
                globalDistanceMM,
                localState.getVelocityMMPerSecond(),
                localState.getAccelerationMMPerSecondSquared(),
                globalProgress
        );
    }

    /**
     * Finds which segment is active at a given global elapsed time.
     *
     * Backward linear scan -- same idiom CompositeDrivePath uses to
     * find its active segment by distance; there are only a handful of
     * trajectory segments, so this is inexpensive and easy to follow.
     */
    private TrajectorySegment findSegmentForTime(double elapsedTimeSeconds) {
        for (int index = segments.size() - 1; index >= 0; index--) {
            TrajectorySegment segment = segments.get(index);

            if (elapsedTimeSeconds >= segment.getStartTimeSeconds()) {
                return segment;
            }
        }

        return segments.get(0);
    }

    @Override
    public double getTotalDistanceMM() {
        return totalDistanceMM;
    }

    @Override
    public double getTotalTimeSeconds() {
        return totalTimeSeconds;
    }

    @Override
    public boolean isFinished(double elapsedTimeSeconds) {
        return elapsedTimeSeconds >= totalTimeSeconds;
    }
}

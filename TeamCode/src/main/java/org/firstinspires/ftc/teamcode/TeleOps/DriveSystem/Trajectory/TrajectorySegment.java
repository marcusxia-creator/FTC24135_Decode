package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Trajectory;

import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path.DrivePath;
import org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile.MotionProfile;

/**
 * Represents one planned segment of a complete drive trajectory.
 *
 * A TrajectorySegment connects:
 *
 *      DrivePath geometry
 *
 * with:
 *
 *      velocity and acceleration constraints
 *
 * and eventually:
 *
 *      the MotionProfile used to travel that segment.
 *
 * Units:
 * - distance: mm
 * - velocity: mm/s
 * - acceleration/deceleration: mm/s^2
 * - time: seconds
 */
public final class TrajectorySegment {

    /*
     * Geometric path for this segment.
     * Examples:
     * - StraightLinePath
     * - CubicBezierPath
     */
    private final DrivePath drivePath;

    /*
     * Maximum allowed motion values for this segment.
     */
    private final double maximumVelocityMMPerSecond;
    private final double maximumAccelerationMMPerSecondSquared;
    private final double maximumDecelerationMMPerSecondSquared;

    /*
     * Velocity at the beginning and end of this segment.
     * These values will normally be assigned by TrajectoryPlanner.
     */
    private double startVelocityMMPerSecond;
    private double endVelocityMMPerSecond;

    /*
     * Location of this segment inside the complete trajectory.
     * Example:
     *
     * Segment 0:
     *      start distance = 0
     *
     * Segment 1:
     *      start distance = length of segment 0
     */
    private double startDistanceAlongTrajectoryMM;
    /*
     * Time when this segment begins in the complete trajectory.
     */
    private double startTimeSeconds;
    /*
     * Motion profile generated for this segment.
     *
     * Initially null.
     *
     * TrajectoryPlanner / TrajectoryProfile will assign this after
     * start and end velocities have been calculated.
     */
    private MotionProfile motionProfile;

    ///Constructor
    public TrajectorySegment(
            DrivePath drivePath,
            double maximumVelocityMMPerSecond,
            double maximumAccelerationMMPerSecondSquared,
            double maximumDecelerationMMPerSecondSquared
    ) {
        if (drivePath == null) {
            throw new IllegalArgumentException(
                    "DrivePath cannot be null."
            );
        }

        if (maximumVelocityMMPerSecond <= 0.0) {
            throw new IllegalArgumentException(
                    "Maximum velocity must be greater than zero."
            );
        }

        if (maximumAccelerationMMPerSecondSquared <= 0.0) {
            throw new IllegalArgumentException(
                    "Maximum acceleration must be greater than zero."
            );
        }

        if (maximumDecelerationMMPerSecondSquared <= 0.0) {
            throw new IllegalArgumentException(
                    "Maximum deceleration must be greater than zero."
            );
        }

        this.drivePath = drivePath;
        this.maximumVelocityMMPerSecond = maximumVelocityMMPerSecond;
        this.maximumAccelerationMMPerSecondSquared = maximumAccelerationMMPerSecondSquared;
        this.maximumDecelerationMMPerSecondSquared = maximumDecelerationMMPerSecondSquared;

        /*
         * Default boundary velocities.
         * TrajectoryPlanner will change these later.
         */
        this.startVelocityMMPerSecond = 0.0;
        this.endVelocityMMPerSecond = 0.0;
        this.startDistanceAlongTrajectoryMM = 0.0;
        this.startTimeSeconds = 0.0;
        this.motionProfile = null;
    }

    public DrivePath getDrivePath() {
        return drivePath;
    }

    public double getLengthMM() {
        return drivePath.getLengthMM();
    }

    public double getMaximumVelocityMMPerSecond() {
        return maximumVelocityMMPerSecond;
    }

    public double getMaximumAccelerationMMPerSecondSquared() {
        return maximumAccelerationMMPerSecondSquared;
    }

    public double getMaximumDecelerationMMPerSecondSquared() {
        return maximumDecelerationMMPerSecondSquared;
    }

    public double getStartVelocityMMPerSecond() {
        return startVelocityMMPerSecond;
    }

    public double getEndVelocityMMPerSecond() {
        return endVelocityMMPerSecond;
    }

    public double getStartDistanceAlongTrajectoryMM() {
        return startDistanceAlongTrajectoryMM;
    }

    public double getEndDistanceAlongTrajectoryMM() {
        return startDistanceAlongTrajectoryMM
                + drivePath.getLengthMM();
    }

    public double getStartTimeSeconds() {
        return startTimeSeconds;
    }

    public MotionProfile getMotionProfile() {
        return motionProfile;
    }

    /**
     * Called by TrajectoryPlanner after it determines the velocity
     * entering this segment.
     */
    public void setStartVelocityMMPerSecond(
            double startVelocityMMPerSecond
    ) {
        this.startVelocityMMPerSecond =
                Math.max(
                        0.0,
                        startVelocityMMPerSecond
                );
    }

    /**
     * Called by TrajectoryPlanner after it determines the velocity
     * leaving this segment.
     */
    public void setEndVelocityMMPerSecond(
            double endVelocityMMPerSecond
    ) {
        this.endVelocityMMPerSecond =
                Math.max(
                        0.0,
                        endVelocityMMPerSecond
                );
    }

    /**
     * Sets this segment's starting distance inside the complete
     * trajectory.
     */
    public void setStartDistanceAlongTrajectoryMM(
            double startDistanceAlongTrajectoryMM
    ) {
        this.startDistanceAlongTrajectoryMM =
                Math.max(
                        0.0,
                        startDistanceAlongTrajectoryMM
                );
    }

    /**
     * Sets this segment's starting time inside the complete trajectory.
     */
    public void setStartTimeSeconds(
            double startTimeSeconds
    ) {
        this.startTimeSeconds =
                Math.max(
                        0.0,
                        startTimeSeconds
                );
    }

    /**
     * Assigns the motion profile generated for this segment.
     */
    public void setMotionProfile(
            MotionProfile motionProfile
    ) {
        if (motionProfile == null) {
            throw new IllegalArgumentException(
                    "MotionProfile cannot be null."
            );
        }

        this.motionProfile =
                motionProfile;
    }

    public boolean hasMotionProfile() {
        return motionProfile != null;
    }

    /**
     * Returns this segment's end time in the complete trajectory.
     *
     * If the profile has not been generated yet, the end time cannot
     * be determined.
     */
    public double getEndTimeSeconds() {
        if (motionProfile == null) {
            return startTimeSeconds;
        }

        return startTimeSeconds
                + motionProfile.getTotalTimeSeconds();
    }
}

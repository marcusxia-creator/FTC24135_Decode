package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile;

/**
 * Represents the planned motion at one specific time.
 *
 * This class describes how far along the path the robot should be
 * and how fast it should be moving.
 *
 * Units:
 * - time: seconds
 * - distance: millimetres
 * - velocity: millimetres per second
 * - acceleration: millimetres per second squared
 */

public class MotionState {
    /**
     * Time measured from the beginning of the motion profile.
     */
    private final double timeSeconds;

    /**
     * Planned physical distance traveled along the active DrivePath.
     */
    private final double distanceAlongPathMM;

    /**
     * Planned velocity along the active DrivePath.
     */
    private final double velocityMMPerSecond;

    /**
     * Planned acceleration along the active DrivePath.
     */
    private final double accelerationMMPerSecondSquared;

    /**
     * Normalized motion-profile progress.
     *
     * 0.0 = beginning
     * 1.0 = complete
     */
    private final double profileProgress;

    public MotionState(
            double timeSeconds,
            double distanceAlongPathMM,
            double velocityMMPerSecond,
            double accelerationMMPerSecondSquared,
            double profileProgress
    ) {
        this.timeSeconds =
                Math.max(0.0, timeSeconds);

        this.distanceAlongPathMM =
                Math.max(0.0, distanceAlongPathMM);

        this.velocityMMPerSecond =
                velocityMMPerSecond;

        this.accelerationMMPerSecondSquared =
                accelerationMMPerSecondSquared;

        this.profileProgress =
                clamp(profileProgress, 0.0, 1.0);
    }

    public double getTimeSeconds() {
        return timeSeconds;
    }

    public double getDistanceAlongPathMM() {
        return distanceAlongPathMM;
    }

    public double getVelocityMMPerSecond() {
        return velocityMMPerSecond;
    }

    public double getAccelerationMMPerSecondSquared() {
        return accelerationMMPerSecondSquared;
    }

    public double getProfileProgress() {
        return profileProgress;
    }

    /**
     * Returns true when this state represents the end of the profile.
     */
    public boolean isProfileComplete() {
        return profileProgress >= 1.0;
    }

    private static double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}

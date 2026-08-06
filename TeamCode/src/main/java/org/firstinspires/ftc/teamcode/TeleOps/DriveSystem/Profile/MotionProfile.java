package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile;

/**
 * Common interface for all one-dimensional drive motion profiles.
 *
 * A MotionProfile controls how the robot progresses along a DrivePath.
 *
 * It does not calculate:
 * - field X or Y position
 * - heading
 * - PID output
 * - mecanum motor power
 *
 * Implementations may include:
 * - TrapezoidalProfile
 * - SCurveProfile
 */

public interface  MotionProfile {

    /**
     * Returns the planned motion state at a specified elapsed time.
     *
     * If elapsedTimeSeconds is greater than the profile duration,
     * the implementation should return the final state:
     *
     * - distance = total distance
     * - velocity = 0
     * - acceleration = 0
     * - progress = 1
     *
     * @param elapsedTimeSeconds time from the beginning of the profile
     * @return planned motion state at that time
     */

    MotionState getMotionState(double elapsedTimeSeconds);

    /**
     * Returns the total distance covered by the profile.
     */
    double getTotalDistanceMM();

    /**
     * Returns the total profile duration.
     */
    double getTotalTimeSeconds();

    /**
     * Returns true when the given elapsed time has reached or
     * exceeded the total profile time.
     */
    boolean isFinished(
            double elapsedTimeSeconds
    );

}

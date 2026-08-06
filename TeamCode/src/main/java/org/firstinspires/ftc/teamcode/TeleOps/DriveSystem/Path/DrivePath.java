package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path;

/**
 * Common interface for all drive paths.
 *
 * A path only describes geometry:
 * - where the robot should be
 * - which direction the path is pointing
 *
 * It does not calculate time, PID output, or motor power.
 */

public interface DrivePath {
    /**
     * Returns one point on the path based on physical distance
     * from the beginning of the path.
     *
     * @param distanceAlongPathMM distance traveled along the path in millimetres
     * @return sampled path point
     */

    DrivePathState getPathState(double distanceAlongPathMM);

    /**
     * Returns the total physical length of the path.
     */
    double getLengthMM();

    double getStartXMM();

    double getStartYMM();

    double getEndXMM();

    double getEndYMM();
}

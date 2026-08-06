package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Represents a straight-line path between two field positions.
 *
 * This class calculates:
 * - total path length
 * - the position at any distance along the path
 * - the constant tangent direction
 *
 * Units:
 * - position: millimetres
 * - distance: millimetres
 */
public final class StraightLinePath implements DrivePath {
    private static final double MINIMUM_PATH_LENGTH_MM = 0.000001;

    private final double startXMM;
    private final double startYMM;

    private final double endXMM;
    private final double endYMM;

    private final double pathLengthMM;

    /*
     * Unit vector pointing from the start toward the end.
     *
     * Example:
     * tangentX = 1, tangentY = 0 means positive X direction.
     */
    private final double tangentX;
    private final double tangentY;

    public StraightLinePath(
            double startXMM,
            double startYMM,
            double endXMM,
            double endYMM
    ) {
        this.startXMM = startXMM;
        this.startYMM = startYMM;
        this.endXMM = endXMM;
        this.endYMM = endYMM;

        double changeInx = endXMM - startXMM;
        double changeIny = endYMM - startYMM;
        pathLengthMM = Math.sqrt(changeInx * changeInx + changeIny * changeIny);

        if (pathLengthMM > MINIMUM_PATH_LENGTH_MM) {
            tangentX = changeInx / pathLengthMM;
            tangentY = changeIny / pathLengthMM;
        } else {
            tangentX = 1.0;
            tangentY = 0.0;
        }
    }

    /**
     * Convenience constructor using FTC Pose2D objects.
     * <p>
     * Heading is not used because this class only describes
     * the X-Y path geometry.
     */
    public StraightLinePath(
            Pose2D startPose,
            Pose2D endPose
    ) {
        this(// this is a convenience constructor,
            // constructor chaining and this call the another constructor
            // and pass the followings to the constructor above
                startPose.getX(DistanceUnit.MM),
                startPose.getY(DistanceUnit.MM),
                endPose.getX(DistanceUnit.MM),
                endPose.getY(DistanceUnit.MM)
        );
    }

    /**
     * Returns the path state at a requested physical distance from
     * the beginning of the path.
     */
    @Override
    public DrivePathState getPathState(double distanceAlongPathMM) {
        if (pathLengthMM <= MINIMUM_PATH_LENGTH_MM) {
            return new DrivePathState(
                    0,
                    1.0,
                    startXMM,
                    startYMM,
                    tangentX,
                    tangentY,
                    0,
                    0
            );
        }

        /*
         * Prevent the requested distance from going before the start
         * or past the end of the path.
         */
        double validDistanceAlongPathMM = clamp(
                distanceAlongPathMM,
                0.0,
                pathLengthMM
        );

        double pathProgress =
                validDistanceAlongPathMM / pathLengthMM;

        /*
         * Move from the starting point along the unit tangent vector.
         */
        double pathXMM =
                startXMM
                        + tangentX * validDistanceAlongPathMM;

        double pathYMM =
                startYMM
                        + tangentY * validDistanceAlongPathMM;

        return new DrivePathState(
                validDistanceAlongPathMM,
                pathProgress,
                pathXMM,
                pathYMM,
                tangentX,
                tangentY,
                0.0,
                0.0
        );
    }

    @Override
    public double getLengthMM() {
        return pathLengthMM;
    }

    @Override
    public double getStartXMM() {
        return startXMM;
    }

    @Override
    public double getStartYMM() {
        return startYMM;
    }

    @Override
    public double getEndXMM() {
        return endXMM;
    }

    @Override
    public double getEndYMM() {
        return endYMM;
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

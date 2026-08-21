package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path;

import static androidx.core.math.MathUtils.clamp;

/**
 * Represents the geometric state of a drive path at one specific
 * distance along that path.
 *
 * This class does not contain time, velocity, acceleration, PID,
 * or motor power information.
 *
 * Units:
 * - distance: millimetres
 * - position: millimetres
 * - heading: radians
 * - curvature: 1 / millimetres
 */

public final class DrivePathState {
    /**
     * Physical distance from the beginning of the path
     */
    private final double distanceAlongPathMM;

    // Normalized completion of the path
    // 0.0 = beginning of path
    // 1.0 = end of path
    private final double pathProgress;
    /**
     * X position of the robot
     * Y position of the robot
     */
    private final double xMM;
    private final double yMM;
    /**
     * Unit tangent vector showing the path's direction of travel.
     * The tangent should normally have a magnitude of 1:
     * tangentX² + tangentY² = 1
     */
    private final double tangentX;
    private final double tangentY;

    /**
     * Direction of the path tangent in radians.
     */
    private final double tangentHeadingRadians;
    /**
     * Signed path curvature.
     * Straight line:
     * curvature = 0
     * Positive and negative values indicate opposite turning
     * directions.
     */
    private final double curvaturePerMM;

    ///Constructor
    public DrivePathState(
            double distanceAlongPathMM,
            double pathProgress,
            double xMM,
            double yMM,
            double tangentX,
            double tangentY,
            double tangentHeadingRadians,
            double curvaturePerMM
    ) {
        this.distanceAlongPathMM = Math.max(0, distanceAlongPathMM);
        this.pathProgress = clamp(pathProgress, 0, 1);
        this.xMM = xMM;
        this.yMM = yMM;
        /*
         * Normalize the tangent vector so every path implementation
         * returns a consistent unit direction.
         */
        double tangentMagnitude =
                Math.sqrt(tangentX * tangentX + tangentY * tangentY);

        if (tangentMagnitude > 1e-9){
            this.tangentX = tangentX / tangentMagnitude;
            this.tangentY = tangentY / tangentMagnitude;
        } else {
            this.tangentX = 1;
            this.tangentY = 0;
        }

        this.tangentHeadingRadians = tangentHeadingRadians;
        this.curvaturePerMM = curvaturePerMM;
    }

    public double getDistanceAlongPathMM() {
        return distanceAlongPathMM;
    }

    public double getPathProgress() {
        return pathProgress;
    }

    public double getXMM() {
        return xMM;
    }

    public double getYMM() {
        return yMM;
    }

    public double getTangentX() {
        return tangentX;
    }

    public double getTangentY() {
        return tangentY;
    }

    public double getTangentHeadingRadians() {
        return tangentHeadingRadians;
    }

    public double getCurvaturePerMM() {
        return curvaturePerMM;
    }

    public boolean isAtPathEnd() {
        return pathProgress >= 1;
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
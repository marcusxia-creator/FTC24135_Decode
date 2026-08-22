package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Path;

/**
 * Represents a smooth curved path between two field positions, defined
 * by a cubic Bezier curve with four control points:
 *
 * P0 - start point
 * P1 - start handle (pulls the curve away from P0)
 * P2 - end handle (pulls the curve away from P3)
 * P3 - end point
 *
 * A cubic Bezier curve has no closed-form arc length, so this class
 * precomputes a chord-length lookup table once in the constructor and
 * uses it to convert a physical distance into the curve's internal
 * parameter t (0 at P0, 1 at P3).
 *
 * This class only knows the four control points. Deriving control
 * points from a start/end pose and a handle length is
 * TrajectoryBuilder's job, not this class's.
 *
 * Units:
 * - position: millimetres
 * - distance: millimetres
 * - curvature: 1 / millimetres
 */
public final class CubicBezierPath implements DrivePath {

    private static final double MINIMUM_PATH_LENGTH_MM = 0.000001;

    /*
     * Number of chord segments used to approximate arc length.
     * More samples means a more accurate distance-to-t mapping at the
     * cost of a little extra constructor work (done once, not per loop).
     */
    private static final int ARC_LENGTH_SAMPLE_SEGMENTS = 100;

    private final double x0MM;
    private final double y0MM;
    private final double x1MM;
    private final double y1MM;
    private final double x2MM;
    private final double y2MM;
    private final double x3MM;
    private final double y3MM;

    private final double pathLengthMM;

    /*
     * Chord-length lookup table.
     *
     * sampleTMM[i] is the Bezier parameter t at sample i.
     * sampleDistanceMM[i] is the physical distance travelled from P0
     * up to sample i.
     *
     * Both arrays have ARC_LENGTH_SAMPLE_SEGMENTS + 1 entries.
     */
    private final double[] sampleT;
    private final double[] sampleDistanceMM;

    public CubicBezierPath(
            double x0MM,
            double y0MM,
            double x1MM,
            double y1MM,
            double x2MM,
            double y2MM,
            double x3MM,
            double y3MM
    ) {
        this.x0MM = x0MM;
        this.y0MM = y0MM;
        this.x1MM = x1MM;
        this.y1MM = y1MM;
        this.x2MM = x2MM;
        this.y2MM = y2MM;
        this.x3MM = x3MM;
        this.y3MM = y3MM;

        sampleT = new double[ARC_LENGTH_SAMPLE_SEGMENTS + 1];
        sampleDistanceMM = new double[ARC_LENGTH_SAMPLE_SEGMENTS + 1];

        sampleT[0] = 0.0;
        sampleDistanceMM[0] = 0.0;

        double previousXMM = x0MM;
        double previousYMM = y0MM;

        for (int index = 1; index <= ARC_LENGTH_SAMPLE_SEGMENTS; index++) {
            double t = (double) index / (double) ARC_LENGTH_SAMPLE_SEGMENTS;

            double currentXMM = evaluateX(t);
            double currentYMM = evaluateY(t);

            double deltaXMM = currentXMM - previousXMM;
            double deltaYMM = currentYMM - previousYMM;

            double chordLengthMM =
                    Math.sqrt(deltaXMM * deltaXMM + deltaYMM * deltaYMM);

            sampleT[index] = t;
            sampleDistanceMM[index] =
                    sampleDistanceMM[index - 1] + chordLengthMM;

            previousXMM = currentXMM;
            previousYMM = currentYMM;
        }

        pathLengthMM = sampleDistanceMM[ARC_LENGTH_SAMPLE_SEGMENTS];
    }

    @Override
    public DrivePathState getPathState(double distanceAlongPathMM) {
        if (pathLengthMM <= MINIMUM_PATH_LENGTH_MM) {
            return new DrivePathState(
                    0,
                    1.0,
                    x0MM,
                    y0MM,
                    1.0,
                    0.0,
                    0.0,
                    0.0
            );
        }

        double validDistanceAlongPathMM = clamp(
                distanceAlongPathMM,
                0.0,
                pathLengthMM
        );

        double t = distanceToParameter(validDistanceAlongPathMM);

        double xMM = evaluateX(t);
        double yMM = evaluateY(t);

        double dxDt = evaluateFirstDerivativeX(t);
        double dyDt = evaluateFirstDerivativeY(t);

        double d2xDt2 = evaluateSecondDerivativeX(t);
        double d2yDt2 = evaluateSecondDerivativeY(t);

        double tangentHeadingRadians = Math.atan2(dyDt, dxDt);

        double speedSquared = dxDt * dxDt + dyDt * dyDt;

        double curvaturePerMM;

        if (speedSquared < MINIMUM_PATH_LENGTH_MM) {
            /*
             * Degenerate point (zero derivative, e.g. a control point
             * coinciding with an endpoint). Curvature is undefined
             * there, so report zero rather than dividing by ~zero.
             */
            curvaturePerMM = 0.0;
        } else {
            curvaturePerMM =
                    (dxDt * d2yDt2 - dyDt * d2xDt2)
                            / Math.pow(speedSquared, 1.5);
        }

        double pathProgress = validDistanceAlongPathMM / pathLengthMM;

        return new DrivePathState(
                validDistanceAlongPathMM,
                pathProgress,
                xMM,
                yMM,
                dxDt,
                dyDt,
                tangentHeadingRadians,
                curvaturePerMM
        );
    }

    /**
     * Converts a physical distance along the curve into the Bezier
     * parameter t, using the precomputed chord-length lookup table.
     *
     * The number of samples is small (ARC_LENGTH_SAMPLE_SEGMENTS), so
     * a simple backward linear scan is inexpensive and easy to read --
     * the same idiom CompositeDrivePath uses to find its active
     * segment.
     */
    private double distanceToParameter(double distanceAlongPathMM) {
        int lowerIndex = 0;

        for (int index = ARC_LENGTH_SAMPLE_SEGMENTS - 1;
             index >= 0;
             index--) {

            if (distanceAlongPathMM >= sampleDistanceMM[index]) {
                lowerIndex = index;
                break;
            }
        }

        int upperIndex = lowerIndex + 1;

        double lowerDistanceMM = sampleDistanceMM[lowerIndex];
        double upperDistanceMM = sampleDistanceMM[upperIndex];

        double intervalLengthMM = upperDistanceMM - lowerDistanceMM;

        double localFraction;

        if (intervalLengthMM > MINIMUM_PATH_LENGTH_MM) {
            localFraction =
                    (distanceAlongPathMM - lowerDistanceMM)
                            / intervalLengthMM;
        } else {
            localFraction = 0.0;
        }

        return sampleT[lowerIndex]
                + localFraction * (sampleT[upperIndex] - sampleT[lowerIndex]);
    }

    /*
     * ============================================================
     * Cubic Bezier math
     * ============================================================
     *
     * B(t)   = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
     * B'(t)  = 3(1-t)^2 (P1-P0) + 6(1-t)t (P2-P1) + 3t^2 (P3-P2)
     * B''(t) = 6(1-t)(P2-2P1+P0) + 6t(P3-2P2+P1)
     *
     * Curvature is parameterization-invariant, so evaluating these
     * directly in terms of t (rather than arc length) still produces
     * a correct 1/mm curvature value.
     */

    private double evaluateX(double t) {
        double oneMinusT = 1.0 - t;

        return oneMinusT * oneMinusT * oneMinusT * x0MM
                + 3.0 * oneMinusT * oneMinusT * t * x1MM
                + 3.0 * oneMinusT * t * t * x2MM
                + t * t * t * x3MM;
    }

    private double evaluateY(double t) {
        double oneMinusT = 1.0 - t;

        return oneMinusT * oneMinusT * oneMinusT * y0MM
                + 3.0 * oneMinusT * oneMinusT * t * y1MM
                + 3.0 * oneMinusT * t * t * y2MM
                + t * t * t * y3MM;
    }

    private double evaluateFirstDerivativeX(double t) {
        double oneMinusT = 1.0 - t;

        return 3.0 * oneMinusT * oneMinusT * (x1MM - x0MM)
                + 6.0 * oneMinusT * t * (x2MM - x1MM)
                + 3.0 * t * t * (x3MM - x2MM);
    }

    private double evaluateFirstDerivativeY(double t) {
        double oneMinusT = 1.0 - t;

        return 3.0 * oneMinusT * oneMinusT * (y1MM - y0MM)
                + 6.0 * oneMinusT * t * (y2MM - y1MM)
                + 3.0 * t * t * (y3MM - y2MM);
    }

    private double evaluateSecondDerivativeX(double t) {
        return 6.0 * (1.0 - t) * (x2MM - 2.0 * x1MM + x0MM)
                + 6.0 * t * (x3MM - 2.0 * x2MM + x1MM);
    }

    private double evaluateSecondDerivativeY(double t) {
        return 6.0 * (1.0 - t) * (y2MM - 2.0 * y1MM + y0MM)
                + 6.0 * t * (y3MM - 2.0 * y2MM + y1MM);
    }

    @Override
    public double getLengthMM() {
        return pathLengthMM;
    }

    @Override
    public double getStartXMM() {
        return x0MM;
    }

    @Override
    public double getStartYMM() {
        return y0MM;
    }

    @Override
    public double getEndXMM() {
        return x3MM;
    }

    @Override
    public double getEndYMM() {
        return y3MM;
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

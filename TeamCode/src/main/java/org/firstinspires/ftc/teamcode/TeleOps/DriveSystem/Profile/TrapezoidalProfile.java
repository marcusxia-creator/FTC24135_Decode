package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Profile;
/**
 * One-dimensional trapezoidal motion profile.
 *
 * The profile controls how far the robot should travel along a DrivePath
 * at any elapsed time.
 *
 * Possible phases:
 *
 * 1. Accelerate
 * 2. Cruise at maximum velocity
 * 3. Decelerate
 *
 * For short distances, the robot may not have enough distance to reach
 * maximum velocity. In that case, this class automatically generates a
 * triangular profile:
 *
 * 1. Accelerate
 * 2. Decelerate
 *
 * The profile can also start and/or end at a nonzero velocity. This is
 * what lets TrajectoryPlanner chain several segments together without
 * the robot fully stopping at every waypoint: segment N's end velocity
 * becomes segment N+1's start velocity.
 *
 * Units:
 * - distance: millimetres
 * - velocity: millimetres per second
 * - acceleration: millimetres per second squared
 * - time: seconds
 */

public final class TrapezoidalProfile implements MotionProfile {

    private static final double MINIMUM_VALUE = 0.000001;

    private final double totalDistanceMM;
    private final double startVelocityMMPerSecond;
    private final double endVelocityMMPerSecond;
    private final double maximumVelocityMMPerSecond;
    private final double maximumAccelerationMMPerSecondSquared;
    private final double maximumDecelerationMMPerSecondSquared;

    /*
     * Actual maximum velocity reached by this profile.
     *
     * For a trapezoidal profile:
     * peakVelocity = requested maximum velocity.
     *
     * For a triangular profile:
     * peakVelocity is lower because the path is too short.
     */
    private final double peakVelocityMMPerSecond;

    /*
     * Duration of each profile phase.
     */
    private final double accelerationTimeSeconds;
    private final double cruiseTimeSeconds;
    private final double decelerationTimeSeconds;
    private final double totalTimeSeconds;

    /*
        * Distance covered during each phase.
     */
    private final double cruiseDistanceMM;
    private final double accelerationDistanceMM;
    private final double decelerationDistanceMM;

    private final boolean triangularProfile;

    /**
     * Full constructor, allowing nonzero start/end velocity so
     * segments can be chained without stopping at every waypoint.
     */
    public TrapezoidalProfile(
            double totalDistanceMM,
            double startVelocityMMPerSecond,
            double endVelocityMMPerSecond,
            double maximumVelocityMMPerSecond,
            double maximumAccelerationMMPerSecondSquared,
            double maximumDecelerationMMPerSecondSquared
    ){
        this.totalDistanceMM = Math.max(0.0,totalDistanceMM);
        this.maximumVelocityMMPerSecond = Math.max(MINIMUM_VALUE,maximumVelocityMMPerSecond);
        this.maximumAccelerationMMPerSecondSquared = Math.max(MINIMUM_VALUE,maximumAccelerationMMPerSecondSquared);
        this.maximumDecelerationMMPerSecondSquared = Math.max(MINIMUM_VALUE,maximumDecelerationMMPerSecondSquared);

        this.startVelocityMMPerSecond =
                clamp(startVelocityMMPerSecond, 0.0, this.maximumVelocityMMPerSecond);

        this.endVelocityMMPerSecond =
                clamp(endVelocityMMPerSecond, 0.0, this.maximumVelocityMMPerSecond);

        /*
         * Handle a zero-distance move.
         */
        if (this.totalDistanceMM <= MINIMUM_VALUE){
            peakVelocityMMPerSecond = 0;
            accelerationTimeSeconds = 0;
            cruiseTimeSeconds = 0;
            decelerationTimeSeconds = 0;
            totalTimeSeconds = 0;
            accelerationDistanceMM = 0.0;
            decelerationDistanceMM = 0.0;
            cruiseDistanceMM = 0;
            triangularProfile = false;
            return;
        }

        double v0 = this.startVelocityMMPerSecond;
        double vEnd = this.endVelocityMMPerSecond;

        /*
         * Distance required to accelerate from v0 to maximum velocity.
         * v² = v0² + 2*a*d
         * d = (v² - v0²) / (2*a)
         */
        double distanceToMaximumVelocityDuringAcceleration =
                (this.maximumVelocityMMPerSecond * this.maximumVelocityMMPerSecond
                        - v0 * v0)
                        / (2.0 * this.maximumAccelerationMMPerSecondSquared);

        /*
         * Distance required to decelerate from maximum velocity down
         * to the requested end velocity.
         */
        double distanceToStopFromMaximumVelocity =
                (this.maximumVelocityMMPerSecond * this.maximumVelocityMMPerSecond
                        - vEnd * vEnd)
                        / (2.0 * this.maximumDecelerationMMPerSecondSquared);

        double minimumDistanceNeededForMaximumVelocity =
                distanceToMaximumVelocityDuringAcceleration
                        + distanceToStopFromMaximumVelocity;

        /*
         * Determine whether there is enough path distance to reach
         * maximum velocity.
         */
        if (minimumDistanceNeededForMaximumVelocity
                >= this.totalDistanceMM
        ){
            triangularProfile = true;
            /*
             * For unequal acceleration and deceleration, and nonzero
             * start/end velocity:
             *
             * accel distance:   dA = (vPeak² - v0²)   / (2 * aAccel)
             * decel distance:   dD = (vPeak² - vEnd²) / (2 * aDecel)
             * total distance:   D  = dA + dD
             *
             * Solving for peak velocity:
             *
             * vPeak² =
             *      2*aAccel*aDecel*D + aDecel*v0² + aAccel*vEnd²
             *      ------------------------------------------------
             *                 aAccel + aDecel
             *
             * (Reduces to the original zero-boundary-velocity formula
             * when v0 = vEnd = 0.)
             */
            double peakVelocitySquared =
                    (2.0 * this.maximumAccelerationMMPerSecondSquared
                            * this.maximumDecelerationMMPerSecondSquared
                            * this.totalDistanceMM
                            + this.maximumDecelerationMMPerSecondSquared * v0 * v0
                            + this.maximumAccelerationMMPerSecondSquared * vEnd * vEnd)
                            / (
                            this.maximumAccelerationMMPerSecondSquared
                                    + this.maximumDecelerationMMPerSecondSquared
                    );

            if (peakVelocitySquared < v0 * v0
                    || peakVelocitySquared < vEnd * vEnd) {
                throw new IllegalArgumentException(
                        "Requested start/end velocities are not "
                                + "reachable within the given distance "
                                + "and acceleration limits."
                );
            }

            peakVelocityMMPerSecond = Math.sqrt(peakVelocitySquared);

            accelerationTimeSeconds =
                    (peakVelocityMMPerSecond - v0)
                            / this.maximumAccelerationMMPerSecondSquared;

            cruiseTimeSeconds = 0;

            decelerationTimeSeconds =
                    (peakVelocityMMPerSecond - vEnd)
                            / this.maximumDecelerationMMPerSecondSquared;

            if (accelerationTimeSeconds < 0.0
                    || decelerationTimeSeconds < 0.0) {
                throw new IllegalArgumentException(
                        "Requested start/end velocities are not "
                                + "reachable within the given distance "
                                + "and acceleration limits."
                );
            }

            accelerationDistanceMM =
                    (peakVelocityMMPerSecond * peakVelocityMMPerSecond
                            - v0 * v0)
                            / (2.0 * this.maximumAccelerationMMPerSecondSquared);

            cruiseDistanceMM = 0.0;

            decelerationDistanceMM =
                    (peakVelocityMMPerSecond * peakVelocityMMPerSecond
                            - vEnd * vEnd)
                            / (2.0 * this.maximumDecelerationMMPerSecondSquared);

        } else {
                triangularProfile = false;

                peakVelocityMMPerSecond = this.maximumVelocityMMPerSecond;

                accelerationTimeSeconds =
                        (peakVelocityMMPerSecond - v0)
                                / this.maximumAccelerationMMPerSecondSquared;

                decelerationTimeSeconds =
                        (peakVelocityMMPerSecond - vEnd)
                                / this.maximumDecelerationMMPerSecondSquared;

                accelerationDistanceMM = distanceToMaximumVelocityDuringAcceleration;

                decelerationDistanceMM = distanceToStopFromMaximumVelocity;

                cruiseDistanceMM = this.totalDistanceMM - accelerationDistanceMM - decelerationDistanceMM;

                cruiseTimeSeconds = cruiseDistanceMM / peakVelocityMMPerSecond;
        }

        totalTimeSeconds = accelerationTimeSeconds + cruiseTimeSeconds + decelerationTimeSeconds;

    }

/**
 * Convenience constructor for a profile that starts and ends at rest,
 * with separate acceleration and deceleration limits.
 */
public TrapezoidalProfile(
        double totalDistanceMM,
        double maximumVelocityMMPerSecond,
        double maximumAccelerationMMPerSecondSquared,
        double maximumDecelerationMMPerSecondSquared
    ) {
    this(
            totalDistanceMM,
            0.0,
            0.0,
            maximumVelocityMMPerSecond,
            maximumAccelerationMMPerSecondSquared,
            maximumDecelerationMMPerSecondSquared
    );
}
/**
 * Convenience constructor that uses the same value for
 * acceleration and deceleration.
 */
public TrapezoidalProfile(
            double totalDistanceMM,
            double maximumVelocityMMPerSecond,
            double accelerationAndDecelerationMMPerSecondSquared
    ) {
        this(
                totalDistanceMM,
                maximumVelocityMMPerSecond,
                accelerationAndDecelerationMMPerSecondSquared,
                accelerationAndDecelerationMMPerSecondSquared
        );
    }
/**
 * Returns the planned motion at a specified elapsed time.
 */
@Override
public MotionState getMotionState(
        double elapsedTimeSeconds
    ) {
    double validTimeSeconds =
            Math.max(0.0, elapsedTimeSeconds);

    /*
     * Zero-distance profile.
     */
    if (totalDistanceMM <= MINIMUM_VALUE) {
        return new MotionState(
                0.0,
                0.0,
                0.0,
                0.0,
                1.0
        );
    }
    /*
     * ========================================================
     * Phase 1: acceleration
     * ========================================================
     */
    if (validTimeSeconds < accelerationTimeSeconds) {
        double acceleration =
                maximumAccelerationMMPerSecondSquared;

        double velocity =
                startVelocityMMPerSecond
                        + acceleration * validTimeSeconds;

        double distance =
                startVelocityMMPerSecond * validTimeSeconds
                        + 0.5* acceleration* validTimeSeconds* validTimeSeconds;

        return createMotionState(
                validTimeSeconds,
                distance,
                velocity,
                acceleration
        );
    }

    double accelerationEndTimeSeconds = accelerationTimeSeconds;

    double cruiseEndTimeSeconds = accelerationTimeSeconds + cruiseTimeSeconds;

    /*
     * ========================================================
     * Phase 2: cruise
     * ========================================================
     *
     * A triangular profile has zero cruise time, so this phase
     * is skipped automatically.
     */
    if (validTimeSeconds < cruiseEndTimeSeconds) {
        double timeInCruiseSeconds =
                validTimeSeconds
                        - accelerationEndTimeSeconds;

        double distance =
                accelerationDistanceMM
                        + peakVelocityMMPerSecond
                        * timeInCruiseSeconds;

        return createMotionState(
                validTimeSeconds,
                distance,
                peakVelocityMMPerSecond,
                0.0
        );
    }

    /*
     * ========================================================
     * Phase 3: deceleration
     * ========================================================
     */
    if (validTimeSeconds < totalTimeSeconds) {
        double timeInDecelerationSeconds =
                validTimeSeconds- cruiseEndTimeSeconds;

        double distanceAtStartOfDecelerationMM =
                accelerationDistanceMM+ cruiseDistanceMM;

        double velocity =
                peakVelocityMMPerSecond- maximumDecelerationMMPerSecondSquared* timeInDecelerationSeconds;

        double distance =
                distanceAtStartOfDecelerationMM +
                        peakVelocityMMPerSecond * timeInDecelerationSeconds
                        - 0.5 * maximumDecelerationMMPerSecondSquared * timeInDecelerationSeconds * timeInDecelerationSeconds;

        return createMotionState(
                validTimeSeconds,
                distance,
                Math.max(0.0, velocity),
                -maximumDecelerationMMPerSecondSquared
        );
    }
        /*
         * ========================================================
         * Profile complete
         * ========================================================
         *
         * Keep the reference at the final path position so PID/LQR
         * can finish correcting the actual robot position. Report
         * the requested end velocity (not always zero) so a segment
         * that hands off into the next one at speed doesn't
         * misreport itself as stopped.
         */
        return new MotionState(
                totalTimeSeconds,
                totalDistanceMM,
                endVelocityMMPerSecond,
                0.0,
                1.0
        );
}

/**
 * Returns the planned motion at a specified elapsed time.
 * create MotionState Data container object
 * */
private MotionState createMotionState(
        double timeSeconds,
        double distanceAlongPathMM,
        double velocityMMPerSecond,
        double accelerationMMPerSecondSquared
) {
    double validDistanceMM =
            clamp(
                    distanceAlongPathMM,
                    0.0,
                    totalDistanceMM
            );

    double profileProgress =
            validDistanceMM / totalDistanceMM;

    return new MotionState(
            timeSeconds,
            validDistanceMM,
            velocityMMPerSecond,
            accelerationMMPerSecondSquared,
            profileProgress
    );
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
    public boolean isFinished(
            double elapsedTimeSeconds
    ) {
        return elapsedTimeSeconds >= totalTimeSeconds;
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

    public double getPeakVelocityMMPerSecond() {
        return peakVelocityMMPerSecond;
    }

    public double getStartVelocityMMPerSecond() {
        return startVelocityMMPerSecond;
    }

    public double getEndVelocityMMPerSecond() {
        return endVelocityMMPerSecond;
    }

    public double getAccelerationTimeSeconds() {
        return accelerationTimeSeconds;
    }

    public double getCruiseTimeSeconds() {
        return cruiseTimeSeconds;
    }

    public double getDecelerationTimeSeconds() {
        return decelerationTimeSeconds;
    }

    public double getAccelerationDistanceMM() {
        return accelerationDistanceMM;
    }

    public double getCruiseDistanceMM() {
        return cruiseDistanceMM;
    }

    public double getDecelerationDistanceMM() {
        return decelerationDistanceMM;
    }

    public boolean isTriangularProfile() {
        return triangularProfile;
    }

    private static double requirePositive(
            double value,
            String valueName
    ) {
        if (!Double.isFinite(value)
                || value <= MINIMUM_VALUE) {

            throw new IllegalArgumentException(
                    valueName
                            + " must be a positive finite value."
            );
        }

        return value;
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

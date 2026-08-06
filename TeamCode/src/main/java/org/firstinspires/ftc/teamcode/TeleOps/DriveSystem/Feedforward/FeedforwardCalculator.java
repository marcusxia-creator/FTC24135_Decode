package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.Feedforward;

/**
 * Calculates feedforward motor output from planned velocity
 * and acceleration.
 *
 * Equation:
 *
 * output = kS * sign(velocity)
 *        + kV * velocity
 *        + kA * acceleration
 *
 * Typical units:
 *
 * velocity:
 *      millimetres per second
 *
 * acceleration:
 *      millimetres per second squared
 *
 * output:
 *      normalized motor power
 *
 * The values of kS, kV, and kA must be tuned so that the final
 * output is appropriate for DcMotor.setPower().
 */
public final class FeedforwardCalculator {

    private double kS;
    private double kV;
    private double kA;

    /**
     * Creates a feedforward calculator.
     *
     * @param kS static-friction feedforward
     * @param kV velocity feedforward
     * @param kA acceleration feedforward
     */
    public FeedforwardCalculator(
            double kS,
            double kV,
            double kA
    ) {
        setCoefficients(kS, kV, kA);
    }

    /**
     * Calculates the feedforward output.
     *
     * @param targetVelocityMMPerSecond planned profile velocity
     * @param targetAccelerationMMPerSecondSquared planned acceleration
     * @return normalized feedforward output
     */
    public double calculate(
            double targetVelocityMMPerSecond,
            double targetAccelerationMMPerSecondSquared
    ) {
        double staticOutput = calculateStaticOutput(
                targetVelocityMMPerSecond,
                targetAccelerationMMPerSecondSquared
        );

        double velocityOutput =
                kV * targetVelocityMMPerSecond;

        double accelerationOutput =
                kA * targetAccelerationMMPerSecondSquared;

        return staticOutput
                + velocityOutput
                + accelerationOutput;
    }

    /**
     * Calculates feedforward and constrains the result to a
     * specified maximum absolute output.
     *
     * This is useful when the feedforward output will be combined
     * with PID correction.
     */
    public double calculateLimited(
            double targetVelocityMMPerSecond,
            double targetAccelerationMMPerSecondSquared,
            double maximumAbsoluteOutput
    ) {
        if (!Double.isFinite(maximumAbsoluteOutput)
                || maximumAbsoluteOutput < 0.0) {
            throw new IllegalArgumentException(
                    "Maximum output must be zero or a positive finite value."
            );
        }

        double output = calculate(
                targetVelocityMMPerSecond,
                targetAccelerationMMPerSecondSquared
        );

        return clamp(
                output,
                -maximumAbsoluteOutput,
                maximumAbsoluteOutput
        );
    }

    /**
     * Updates kS, kV, and kA.
     *
     * This can be called during the TeleOp loop when using
     * FTC Dashboard tuning variables.
     */
    public void setCoefficients(
            double kS,
            double kV,
            double kA
    ) {
        requireFinite(kS, "kS");
        requireFinite(kV, "kV");
        requireFinite(kA, "kA");

        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public double getKS() {
        return kS;
    }

    public double getKV() {
        return kV;
    }

    public double getKA() {
        return kA;
    }

    /**
     * Calculates the static-friction term.
     *
     * Velocity direction is preferred. At the beginning of a
     * profile, velocity may still be zero while acceleration is
     * nonzero, so acceleration direction is used as a fallback.
     */
    private double calculateStaticOutput(
            double targetVelocityMMPerSecond,
            double targetAccelerationMMPerSecondSquared
    ) {
        if (Math.abs(targetVelocityMMPerSecond) > 0.001) {
            return kS * Math.signum(
                    targetVelocityMMPerSecond
            );
        }

        if (Math.abs(
                targetAccelerationMMPerSecondSquared
        ) > 0.001) {
            return kS * Math.signum(
                    targetAccelerationMMPerSecondSquared
            );
        }

        return 0.0;
    }

    private static void requireFinite(
            double value,
            String valueName
    ) {
        if (!Double.isFinite(value)) {
            throw new IllegalArgumentException(
                    valueName + " must be a finite value."
            );
        }
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
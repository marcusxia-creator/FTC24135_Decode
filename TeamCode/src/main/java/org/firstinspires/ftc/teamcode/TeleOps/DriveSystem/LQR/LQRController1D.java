package org.firstinspires.ftc.teamcode.TeleOps.DriveSystem.LQR;

/**
 * Closed-form Linear-Quadratic-Regulator state feedback for one axis
 * (X, Y, or heading) modeled as a double integrator:
 *
 * state:   e = [positionError, velocityError]
 * plant:   A = [[0,1],[0,0]]      B = [[0],[1]]
 * control: u = acceleration correction
 * cost:    J = integral of (qPosition*e_p^2 + qVelocity*e_v^2 + rControl*u^2) dt
 *
 * Solving the continuous algebraic Riccati equation for this specific
 * 2-state / 1-input system produces a closed-form gain, so no matrix
 * library or numeric Riccati solve is needed:
 *
 * Kp = sqrt(qPosition / rControl)
 * Kd = sqrt(2 * Kp + qVelocity / rControl)
 *
 * u  = Kp * positionError + Kd * velocityError
 *
 * Unlike PID, this controller has no memory (no integral accumulator),
 * so there is nothing to reset between moves. The velocity term uses a
 * directly measured velocity error instead of a numerically
 * differentiated position error, which is the main practical advantage
 * over a classic PID D-term.
 */
public final class LQRController1D {

    private double qPosition;
    private double qVelocity;
    private double rControl;

    private double kp;
    private double kd;

    public LQRController1D(
            double qPosition,
            double qVelocity,
            double rControl
    ) {
        setCoefficients(qPosition, qVelocity, rControl);
    }

    /**
     * Returns the optimal control correction for the given position
     * and velocity error.
     */
    public double calculate(
            double positionError,
            double velocityError
    ) {
        return kp * positionError + kd * velocityError;
    }

    /**
     * Recomputes Kp/Kd from Q/R weights.
     *
     * Allows FTC Dashboard gain changes to take effect while running.
     */
    public void setCoefficients(
            double qPosition,
            double qVelocity,
            double rControl
    ) {
        requireFinite(qPosition, "qPosition");
        requireFinite(qVelocity, "qVelocity");
        requireFinite(rControl, "rControl");

        if (rControl <= 0.0) {
            throw new IllegalArgumentException(
                    "rControl must be a positive finite value."
            );
        }

        if (qPosition < 0.0 || qVelocity < 0.0) {
            throw new IllegalArgumentException(
                    "qPosition and qVelocity must not be negative."
            );
        }

        this.qPosition = qPosition;
        this.qVelocity = qVelocity;
        this.rControl = rControl;

        this.kp = Math.sqrt(qPosition / rControl);

        this.kd = Math.sqrt(
                2.0 * kp + qVelocity / rControl
        );
    }

    public double getQPosition() {
        return qPosition;
    }

    public double getQVelocity() {
        return qVelocity;
    }

    public double getRControl() {
        return rControl;
    }

    public double getKp() {
        return kp;
    }

    public double getKd() {
        return kd;
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
}

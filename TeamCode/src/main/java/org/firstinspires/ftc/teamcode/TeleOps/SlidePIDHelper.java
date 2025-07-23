package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

public class SlidePIDHelper {
    private final DcMotor[] motors;
    private final PIDController pid;
    private final double maxTicks;

    /**
     * @param motors         One or more slide motors (e.g., left/right)
     * @param kp             Proportional gain
     * @param ki             Integral gain
     * @param kd             Derivative gain
     * @param maxTravelTicks Full-range encoder ticks for normalization (<=0 to disable)
     */
    public SlidePIDHelper(DcMotor[] motors,
                          double kp, double ki, double kd,
                          double maxTravelTicks,double tolerance) {
        this.motors = motors;
        // Reset and prepare
        for (DcMotor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.pid = new PIDController(kp, ki, kd);
        this.pid.setTolerance(tolerance);
        this.maxTicks = maxTravelTicks;
    }

    /**
     * Set target in raw ticks (or normalized if maxTicks>0)
     */
    public void setTargetTicks(double targetTicks) {
        if (maxTicks > 0) {
            pid.setSetPoint(targetTicks / maxTicks);
        } else {
            pid.setSetPoint(targetTicks);
        }
    }

    /**
     * Convenience to set target by linear units (e.g., mm or inches)
     */
    public void setTargetInches(double inches) {
        setTargetTicks(mmToTicks(inches));
    }

    /**
     * Call each loop to update motor power
     */
    public void update() {
        // Average current position across motors
        double sum = 0;
        for (DcMotor m : motors) sum += m.getCurrentPosition();
        double avgPos = sum / motors.length;

        // Normalize if requested
        double measurement = (maxTicks > 0) ? avgPos / maxTicks : avgPos;

        // Compute PID
        double power = pid.calculate(measurement);
        power = Range.clip(power, -1.0, 1.0);

        // Apply to all motors
        for (DcMotor m : motors) {
            m.setPower(power);
        }
    }

    /**
     * @return true if within tolerance of setpoint
     */
    public boolean atTarget() {
        return pid.atSetPoint();
    }

    // Convert linear inches to ticks; adjust counts and diameter
    private double mmToTicks(double dist) {
        ///dist in mm unit
        return (dist * RobotActionConfig.TICKS_PER_MM_SLIDES);
    }
}

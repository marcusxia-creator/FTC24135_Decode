package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SlidesPIDControl {
    private final RobotHardware robot;
    private final PIDController pid;
    private boolean enabled = true;                                                                 // default ON

    /// Travel.units
    private final double maxTicks;                                                                  //full stroke in ticks
    private final double ticksPerMM;                                                                // conversion factor

    /// Targets
    private double targetTicksRaw = 0.0;                                                            // target in raw ticks

    /// Feedforward terms
    private final double fu;
    private final double fd; // your original "f" — directional bias
    private double ff = 0;                                                                          // ff is the dynamic feedforward based on the slide up or down

    //Optional baseline gravity term (set to 0 if you don't want it)
    private double kG = 0.0;
    private double kV = 0.0;

    /// motor power output
    private double power = 0.0;

    // Velocity deadband so direction doesn't chatter around 0
    private double V_DEADBAND_TPS = 12.0;                                                           // ticks/sec

    // ---- Motion profile (trapezoid/triangle, start&end vel ~ 0) ----
    public double vMaxTps = 1000.0;      // max velocity (ticks/s)  <-- tune
    public double aMaxTps2 = 3000.0;     // max accel   (ticks/s^2) <-- tune

    private final ElapsedTime profileTimer = new ElapsedTime();
    private double profStartPos = 0.0;   // ticks
    private double profDelta = 0.0;      // target - start (ticks)
    private double profSign = 1.0;       // +1 up, -1 down
    private double t1 = 0, t2 = 0, t3 = 0, tTotal = 0; // phase times (s)
    private boolean profileActive = false;

    // ---- For velocity calculation ---
    private final ElapsedTime runtime = new ElapsedTime();
    private int lastAvgPos = 0;
    private double lastTimeSec = 0.0;

    /**
     * @param robot            Your RobotHardware instance (contains liftMotorLeft & liftMotorRight)
     * @param kp               Proportional gain
     * @param ki               Integral gain
     * @param kd               Derivative gain
     * @param maxTravelTicks   Full-range encoder ticks for normalization (<=0 to disable)
     * @param ticksPerMM       Ticks for per mm rotation
     */

    public SlidesPIDControl(RobotHardware robot,
                            double kp, double ki, double kd, double fu,double fd,
                            double maxTravelTicks, double ticksPerMM) {
        this.robot = robot;
        this.ticksPerMM = ticksPerMM;
        this.maxTicks = maxTravelTicks;

        // Reset and configure both lift motors
        for (DcMotor m : new DcMotor[]{ robot.liftMotorLeft, robot.liftMotorRight }) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        this.pid = new PIDController(kp, ki, kd);
        this.fu = fu;
        this.fd = fd;

        // Tolerance: normalized if maxTicks provided, else raw ticks
        if (maxTicks > 0) {
            pid.setTolerance(RobotActionConfig.slideTickThreshold / maxTicks);
        } else {
            pid.setTolerance(RobotActionConfig.slideTickThreshold);
        }

        runtime.reset();
        lastAvgPos = getAvgPos();
        lastTimeSec = runtime.seconds();

        // initialize profile to current pos (no motion)
        buildProfile(getAvgPos(), getAvgPos());
    }

    /** Optional: baseline gravity hold term */
    public void setGravityFF(double kG) { this.kG = kG; }

    public void setVelocityFF(double kV) { this.kV = kV; }

    public void setVmaxTps(double v) { this.vMaxTps = Math.max(1, v); }
    public void setAmaxTps2(double a) { this.aMaxTps2 = Math.max(1, a); }

    /** Optional: velocity deadband tuning */
    public void setVelocityDeadbandTps(double deadband) { this.V_DEADBAND_TPS = Math.max(0, deadband); }

    /** Set target position in encoder ticks (normalized if maxTicks>0) */
    public void setTargetTicks(double targetTicks) {
        this.targetTicksRaw = targetTicks;
        if (maxTicks > 0) {
            pid.setSetPoint(targetTicks / maxTicks);
        } else {
            pid.setSetPoint(targetTicks);
            pid.setTolerance(RobotActionConfig.slideTickThreshold);
        }
        pid.reset(); // avoid surge on new targets
        // Build a new trapezoidal/triangular profile from current position to target
        buildProfile(getAvgPos(), targetTicks);
    }

    /**
     * Convenience to set target by linear units (e.g., mm or inches)
     */
    public void setTargetMM(double mm) {
        setTargetTicks(mmToTicks(mm));
    }

    /** Build a fresh trapezoid/triangle from currentPos -> newTarget (start/end vel ~0). */
    private void buildProfile(double currentPosTicks, double targetTicks) {
        profStartPos = currentPosTicks;
        profDelta = targetTicks - currentPosTicks;
        profSign = (profDelta >= 0) ? 1.0 : -1.0;
        double d = Math.abs(profDelta);

        if (d < 1e-4) {
            // no motion
            t1 = t2 = t3 = tTotal = 0;
            profileActive = false;
            profileTimer.reset();
            return;
        }

        // Check if we can hit vMax; else use triangular with peak v = sqrt(d * aMax)
        double tAccelToVmax = vMaxTps / aMaxTps2;
        double dAccel = 0.5 * aMaxTps2 * tAccelToVmax * tAccelToVmax;
        double dNeededForAccelDecel = 2.0 * dAccel;

        if (d <= dNeededForAccelDecel) {
            // Triangular profile
            double vPeak = Math.sqrt(d * aMaxTps2);
            t1 = vPeak / aMaxTps2;   // accel
            t2 = 0.0;                // cruise
            t3 = t1;                 // decel
        } else {
            // Trapezoid profile
            t1 = tAccelToVmax;                       // accel to vMax
            t3 = t1;                                 // decel from vMax
            double dCruise = d - dNeededForAccelDecel;
            t2 = dCruise / vMaxTps;                  // cruise duration
        }
        tTotal = t1 + t2 + t3;
        profileActive = true;
        profileTimer.reset();
    }
    /**
     * Call from FSM to enable/disable PID updates
     */
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /**
     * Call once per loop to update motor powers
     */
    public void update() {
        if(!enabled){
            return;
        }
        // Average encoder positions
        int avgPos = getAvgPos();

        double t = profileTimer.seconds();
        double xSet, vSet;  // ticks, ticks/s (signed)

        if (!profileActive || t >= tTotal) {
            // End of profile (or no motion): hold final
            xSet = profStartPos + profDelta;
            vSet = 0.0;
            profileActive = false;
        } else {
            // piecewise
            if (t <= t1) {
                // accelerate: v = a * t, x = 0.5 * a * t^2
                vSet = aMaxTps2 * t;
                xSet = 0.5 * aMaxTps2 * t * t;
            } else if (t <= t1 + t2) {
                // cruise
                double tc = t - t1;
                vSet = vMaxTps;
                xSet = 0.5 * aMaxTps2 * t1 * t1 + vMaxTps * tc;
            } else {
                // decel: mirror of accel
                double td = t - t1 - t2;
                double v = vMaxTps - aMaxTps2 * td;
                vSet = Math.max(0, v);
                double dAccel = 0.5 * aMaxTps2 * t1 * t1;
                double dCruise = vMaxTps * t2;
                double dDecel = vMaxTps * td - 0.5 * aMaxTps2 * td * td;
                xSet = dAccel + dCruise + dDecel;
            }
            // apply direction and start offset
            vSet *= profSign;
            xSet = profStartPos + profSign * xSet;
        }

        // PID measurement & setpoint (normalized if requested)
        double measurement = (maxTicks > 0) ? (avgPos / maxTicks) : avgPos;
        double setpoint    = (maxTicks > 0) ? (xSet   / maxTicks) : xSet;

        // Compute PID output
        double pidOut = pid.calculate(measurement, setpoint);

        // ---- Feedforward: kG + static (fu/fd from desired velocity) + kV*vSet ----
        double sign;
        if (vSet >  V_DEADBAND_TPS)      sign = +1.0;
        else if (vSet < -V_DEADBAND_TPS) sign = -1.0;
        else                              sign =  0.0;

        ff = kG + (sign > 0 ? fu : (sign < 0 ? fd : 0.0)) + (kV * vSet);

        // Compute motor power
        power =pidOut+ff;

        power = Range.clip(power, -1.0, 1.0);

        // Apply to both motors
        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);

        // Save for next loop
        lastAvgPos = avgPos;

        lastTimeSec = runtime.seconds();
    }

    /**
     * @return true if within tolerance of target
     */
    public boolean atTarget() {
        return pid.atSetPoint();
    }

    private int getAvgPos() {
        int left = robot.liftMotorLeft.getCurrentPosition();
        int right = robot.liftMotorRight.getCurrentPosition();
        return (left + right) / 2;
    }

    /// Convert linear inches to encoder ticks; adjust counts and diameter
    private double mmToTicks(double mm) {
        return mm * RobotActionConfig.TICKS_PER_MM_SLIDES;
    }

    /** Optional helper to read current target in ticks. */
    public double getTargetTicks() {
        return targetTicksRaw;
    }

    public double getpower(){return power;}
    public double getf(){return ff;}

}

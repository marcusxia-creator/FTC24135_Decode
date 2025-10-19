package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
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
    private double ff = 0;                                                                          // ff is the dynamic feedforward based on the slide up or down

    // ---- Feedforward Parameters ----
    public double kG = 0.08;       // gravity hold
    public double kSup = 0.040;    // static up
    public double kSdown = 0.030;  // static down
    public double kV = 0.00020;    // ticks/s -> power
    public double kA = 0.00002;    // ticks/s^2 -> power
    // Velocity deadband so direction doesn't chatter around 0
    public double velDeadband = 12; // ticks/s (avoid flip near 0)
    public double smoothVel = 30;   // ticks/s (tanh smoothing)
    public double thetaRad = 10.0;   // 0 for vertical slides
    // Optional hysteresis latch for static term
    private int lastDir = 0;        // -1, 0, +1
    public boolean useHysteresis = false;  // flip on if you still see micro-flips

    /// motor power output
    private double power = 0.0;

    // ---- Motion profile (trapezoid/triangle, start&end vel ~ 0) ----
    public double vMaxTps = 2200;               // max velocity (ticks/s)  <-- tune
    public double aMaxTps2 = 2200;              // max accel   (ticks/s^2) <-- tune

    private final ElapsedTime profileTimer = new ElapsedTime();
    private double profStartPos = 0.0;          // ticks
    private double profDelta = 0.0;             // target - start (ticks)
    private double profSign = 1.0;              // +1 up, -1 down
    private double t1 = 0, t2 = 0, t3 = 0, tTotal = 0; // phase times (s)
    private boolean profileActive = false;

    // ---- For velocity calculation ---
    private final ElapsedTime runtime = new ElapsedTime();
    private int lastAvgPos = 0;
    private double lastTimeSec = 0.0;

    // Tunables (put @Config if you use Dashboard)
    public int minMoveTicks = 5;                    // don't build a profile for shorter moves
    public double velStopTps = 15.0;                // "stopped" if |velocity| below this
    public double settleHoldSec = 0.15;             // time to remain stopped to count as settled

    private final ElapsedTime settleTimer = new ElapsedTime();

    /**
     * @param robot            Your RobotHardware instance (contains liftMotorLeft & liftMotorRight)
     * @param kp               Proportional gain
     * @param ki               Integral gain
     * @param kd               Derivative gain
     * @param maxTravelTicks   Full-range encoder ticks for normalization (<=0 to disable)
     * @param ticksPerMM       Ticks for per mm rotation
     *
     */

    public SlidesPIDControl(RobotHardware robot,
                            double kp, double ki, double kd,
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

    /*** Call from FSM to enable/disable PID updates*/
    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    /** Set target position in encoder ticks (normalized if maxTicks>0) */
    public void setTargetTicks(double targetTicks) {
        this.targetTicksRaw = targetTicks;
        buildProfile(getAvgPos(),targetTicks);
        if (maxTicks > 0) {
            pid.setSetPoint(targetTicks / maxTicks);
        } else {
            pid.setSetPoint(targetTicks);
            pid.setTolerance(RobotActionConfig.slideTickThreshold);
        }
        pid.reset(); // avoid surge on new targets
    }

    /** * Convenience to set target by linear units (e.g., mm or inches) */
    public void setTargetMM(double mm) {
        setTargetTicks(mmToTicks(mm));
    }

    /** Build a fresh trapezoid/triangle from currentPos -> newTarget (start/end vel ~0). */
    private void buildProfile(double currentPosTicks, double targetTicks) {
        profStartPos = currentPosTicks;
        profDelta = targetTicks - currentPosTicks;
        profSign = (profDelta >= 0) ? 1.0 : -1.0;
        double d = Math.abs(profDelta);

        if (d <=minMoveTicks) {
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
    /** using dir of slide moving */
    /** * Feedforward Helper  * Signed velocity/acceleration in ticks/s and ticks/s^2 */
    private double feedforward(double vTps, double aTps2) {
        // 1) Gravity hold (include cos(theta) in case slide is tilted)
        double ff = kG * Math.cos(thetaRad);

        // 2) Static term: smooth sign + smooth magnitude (no hard step at deadband)
        // Direction source: either soft sign from tanh, or latched hysteresis if enabled.
        double softSign = Math.tanh(vTps / Math.max(1e-6, smoothVel));   // [-1,1]
        int dir = useHysteresis ? latchedDir(vTps, velDeadband) : 0;

        // Blend-in factor: 0 at deadband, ~1 by ~2× deadband
        double blend = smoothStep(Math.abs(vTps), velDeadband, 2.0 * velDeadband);

        double stat;
        if (useHysteresis) {
            // Use latched direction when near zero
            stat = (dir > 0 ? kSup : (dir < 0 ? -kSdown : 0.0)) * blend;
        } else {
            // Use soft sign (continuous)
            stat = (softSign > 0 ? kSup : (softSign < 0 ? -kSdown : 0.0)) * blend;
        }

        // 3) Velocity + acceleration terms (proportional FF)
        double velTerm = kV * vTps;     // ticks/s -> power
        double accTerm = kA * aTps2;    // ticks/s^2 -> power

        double baseFF = ff + stat + velTerm + accTerm;

        // 4) Battery voltage compensation (optional but recommended)
        return baseFF * getVoltageComp();
    }

    /*** Call once per loop to update motor powers  */
    public void update() {
        if(!enabled){
            return;
        }
        // only use one motor encoder to control both motors
        int avgPos = robot.liftMotorRight.getCurrentPosition();

        // --- Measured velocity (ticks/s) for "stopped" detection ---
        double nowSec = runtime.seconds();
        double dt = Math.max(1e-3, nowSec - lastTimeSec);
        double vMeasTps = (avgPos - lastAvgPos) / dt;

        // Maintain settle timer: count how long we've been "stopped"
        boolean stoppedNow = Math.abs(vMeasTps) < velStopTps;
        if (stoppedNow) {
            // let timer run while we’re stopped
        } else {
            settleTimer.reset(); // moving again -> reset the "stopped" duration
        }

        // profiler
        double t = profileTimer.seconds();
        double xSet, vSet, aSet;  // ticks, ticks/s (signed)

        if (!profileActive || t >= tTotal) {
            // End of profile (or no motion): hold final
            xSet = profStartPos + profDelta;
            vSet = 0.0;
            aSet = 0.0;
            profileActive = false;
        } else {
            // piecewise
            if (t <= t1) {
                double ta = t;
                double v = aMaxTps2 * ta;          // unsigned speed
                double x = 0.5 * aMaxTps2 * ta * ta;

                vSet = v * profSign;
                xSet = profStartPos + x * profSign;
                aSet = aMaxTps2 * profSign;        // signed accel
            } else if (t <= t1 + t2) {
                // cruise
                double tc = t - t1;
                double dAccel = 0.5 * aMaxTps2 * t1 * t1;

                vSet = vMaxTps * profSign;
                xSet = profStartPos + (dAccel + vMaxTps * tc) * profSign;
                aSet = 0.0;
            } else {
                // decel: mirror of accel
                double td = t - t1 - t2;
                double v = Math.max(0, vMaxTps - aMaxTps2 * td);   // unsigned speed, non-negative
                double dAccel = 0.5 * aMaxTps2 * t1 * t1;
                double dCruise = vMaxTps * t2;
                double dDecel  = vMaxTps * td - 0.5 * aMaxTps2 * td * td;

                vSet = v * profSign;
                xSet = profStartPos + (dAccel + dCruise + dDecel) * profSign;
                aSet = -aMaxTps2 * profSign;                        // signed decel
            }
        }

        /// --- Optional "snap" when very close and basically stopped ---
        int finalTarget = (int)Math.round(profStartPos + profDelta);
        //int finalTarget =getAvgPos();
        int errToFinal = finalTarget - avgPos;

        if (!profileActive) {
            // If profile says we're done but there's a tiny residual, zero setpoints
            if (Math.abs(errToFinal) <= minMoveTicks && Math.abs(vMeasTps) < velStopTps) {
                xSet = finalTarget;
                vSet = 0.0;
                aSet = 0.0;
            }
        }

        // PID measurement & setpoint (normalized if requested)
        double measurement = (maxTicks > 0) ? (avgPos / maxTicks) : avgPos;
        double setpoint    = (maxTicks > 0) ? (xSet   / maxTicks) : xSet;

        // Compute PID output
        double pidOut = pid.calculate(measurement, setpoint);

        // Calculate feedforward
        ff = feedforward(vSet,aSet);           // uses kG, kSup/kSdown, kv*vSet, kA*aSet

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
        // profile inactive AND PID within tolerance AND settled
        boolean pidOk = pid.atSetPoint(); // make sure you set tolerance appropriately
        return !profileActive && pidOk && (settleTimer.seconds() >= settleHoldSec);
    }

    private int getAvgPos() {
        return (robot.liftMotorRight.getCurrentPosition());
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

    // call this each loop or cache periodically
    private double getVoltageComp() {
        double v = robot.getBatteryVoltageRobust(); // implement in your RobotHardware
        return (v > 6.0) ? (12.0 / v) : 1.0; // scale relative to 12V nominal
    }

    /** Smoothstep 0->1 between [a,b] to fade in static FF with no step jump. */
    private static double smoothStep(double x, double a, double b) {
        double t = Math.max(0, Math.min(1, (x - a) / Math.max(1e-9, b - a)));
        return t * t * (3 - 2 * t);
    }

    /** Optional direction latch to prevent tiny sign flips around zero velocity. */
    private int latchedDir(double vTps, double band) {
        if (vTps >  band) lastDir = +1;
        else if (vTps < -band) lastDir = -1;
        return lastDir;
    }
}

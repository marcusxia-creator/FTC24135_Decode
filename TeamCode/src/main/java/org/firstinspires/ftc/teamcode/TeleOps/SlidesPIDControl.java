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

    /// motor power output
    private double power = 0.0;

    // Velocity deadband so direction doesn't chatter around 0
    private double V_DEADBAND_TPS = 12.0;                                                           // ticks/sec

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
    }

    /** Optional: baseline gravity hold term */
    public void setGravityFF(double kG) { this.kG = kG; }

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
    }

    /**
     * Convenience to set target by linear units (e.g., mm or inches)
     */
    public void setTargetMM(double mm) {
        setTargetTicks(mmToTicks(mm));
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
        //double leftPos  = robot.liftMotorLeft.getCurrentPosition();
        //double rightPos = robot.liftMotorRight.getCurrentPosition();
        //double avgPos   = (leftPos + rightPos) / 2.0;
        int avgPos = robot.liftMotorRight.getCurrentPosition();
        // Normalize if requested
        double measurement = (maxTicks > 0) ? avgPos / maxTicks : avgPos;

        // Compute PID output
        double pidOut = pid.calculate(measurement);

        // ---- Velocity-based FF direction with deadband ----
        double now = runtime.seconds();
        double dt = now - lastTimeSec;
        if (dt < 1e-4) dt = 1e-3; // avoid div-by-zero at start

        double avgVelTicksPerSec = (avgPos - lastAvgPos) / dt;

        double sign;
        if (avgVelTicksPerSec >  V_DEADBAND_TPS)      sign = +1.0;   // moving up
        else if (avgVelTicksPerSec < -V_DEADBAND_TPS) sign = -1.0;   // moving down
        else                                          sign =  0.0;   // basically stopped

        // Baseline gravity + directional static FF (no flip-flop near zero due to deadband)
        ff = kG + (sign > 0 ? fu : (sign < 0 ? fd : 0.0));

        // Compute motor power
        power =pidOut+ff;

        power = Range.clip(power, -1.0, 1.0);

        // Apply to both motors
        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);

        // Save for next loop
        lastAvgPos = avgPos;
        lastTimeSec = now;
    }

    /**
     * @return true if within tolerance of target
     */
    public boolean atTarget() {
        return pid.atSetPoint();
    }

    private int getAvgPos() {
        return (robot.liftMotorRight.getCurrentPosition()) / 2;
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

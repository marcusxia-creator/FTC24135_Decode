package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class SlidesPIDControl {
    private final RobotHardware robot;
    private final PIDController pid;
    private final double maxTicks;

    /**
     * @param robot            Your RobotHardware instance (contains liftMotorLeft & liftMotorRight)
     * @param kp               Proportional gain
     * @param ki               Integral gain
     * @param kd               Derivative gain
     * @param maxTravelTicks   Full-range encoder ticks for normalization (<=0 to disable)
     */
    public SlidesPIDControl(RobotHardware robot,
                          double kp, double ki, double kd,
                          double maxTravelTicks) {
        this.robot = robot;
        // Reset and configure both lift motors
        for (DcMotor m : new DcMotor[]{ robot.liftMotorLeft, robot.liftMotorRight }) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.pid = new PIDController(kp, ki, kd);
        this.pid.setTolerance(5);
        this.maxTicks = maxTravelTicks;
    }

    /**
     * Set target position in encoder ticks (normalized if maxTicks>0)
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
     * Call once per loop to update motor powers
     */
    public void update() {
        // Average encoder positions
        double leftPos  = robot.liftMotorLeft.getCurrentPosition();
        double rightPos = robot.liftMotorRight.getCurrentPosition();
        double avgPos   = (leftPos + rightPos) / 2.0;

        // Normalize if requested
        double measurement = (maxTicks > 0) ? avgPos / maxTicks : avgPos;

        // Compute PID output
        double power = pid.calculate(measurement);
        power = Range.clip(power, -1.0, 1.0);

        // Apply to both motors
        robot.liftMotorLeft.setPower(power);
        robot.liftMotorRight.setPower(power);
    }

    /**
     * @return true if within tolerance of target
     */
    public boolean atTarget() {
        return pid.atSetPoint();
    }

    // Convert linear inches to encoder ticks; adjust counts and diameter
    private double mmToTicks(double mm) {
        return mm * RobotActionConfig.TICKS_PER_MM_SLIDES;
    }
}

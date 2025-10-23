package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * PID + Motion Profile Mecanum Drive Controller
 * Integrates PID correction with trapezoidal feedforward.
 *
 * Units: mm, radians internally
 */
@Config
public class PIDDriveToPoint {

    private final RobotHardware robot;
    private final double ticksPerMM;
    private final double maxTicks;

    // PID controllers
    private final PIDController pidX;
    private final PIDController pidY;
    private final PIDController pidH;

    // Motion profile parameters
    public static double MAX_VEL_MM_S = 900;     // mm/s
    public static double MAX_ACC_MM_S2 = 1800;   // mm/s^2

    // Feedforward constants
    public static double kV = 0.020;   // velocity feedforward gain
    public static double kS = 0.120;   // static friction offset

    // PID gains
    public static double pGain = 0.008;
    public static double dGain = 0.00001;
    public static double yawPGain = 5.0;
    public static double yawDGain = 0.0;

    // Tolerances
    public static double xyToleranceMM = 12;
    public static double headingToleranceDeg = 1.5;

    // Internal timers
    private final ElapsedTime profileTimer = new ElapsedTime();
    private final ElapsedTime settleTimer = new ElapsedTime();

    // Motion profile state
    private MotionProfile profile;
    private boolean profileActive = false;

    // Poses
    private Pose2D startPose;
    private Pose2D targetPose;
    private Pose2D currentPose;

    public PIDDriveToPoint(RobotHardware robot, double ticksPerMM, double maxTicks) {
        this.robot = robot;
        this.ticksPerMM = ticksPerMM;
        this.maxTicks = maxTicks;

        pidX = new PIDController(pGain, 0, dGain);
        pidY = new PIDController(pGain, 0, dGain);
        pidH = new PIDController(yawPGain, 0, yawDGain);

        pidX.setTolerance(xyToleranceMM);
        pidY.setTolerance(xyToleranceMM);
        pidH.setTolerance(headingToleranceDeg);
    }

    /** Start a new motion toward target */
    public void setTarget(Pose2D currentPose, Pose2D targetPose) {
        this.startPose = currentPose;
        this.targetPose = targetPose;
        this.currentPose = currentPose;

        // Plan motion profile distance
        double dx = targetPose.getX(DistanceUnit.MM) - startPose.getX(DistanceUnit.MM);
        double dy = targetPose.getY(DistanceUnit.MM) - startPose.getY(DistanceUnit.MM);
        double distance = Math.hypot(dx, dy);

        this.profile = new MotionProfile(distance, MAX_VEL_MM_S, MAX_ACC_MM_S2);
        profileTimer.reset();
        profileActive = true;
        settleTimer.reset();

        // PID setpoints
        pidX.setSetPoint(targetPose.getX(DistanceUnit.MM));
        pidY.setSetPoint(targetPose.getY(DistanceUnit.MM));
        pidH.setSetPoint(targetPose.getHeading(AngleUnit.DEGREES));
    }

    /** Update robot motion each control loop */
    public void update(Pose2D currentPose) {
        this.currentPose = currentPose;
        if (targetPose == null || !profileActive) return;

        // Compute vector to target
        double dx = targetPose.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM);
        double dy = targetPose.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM);
        double distance = Math.hypot(dx, dy);
        double headingToTarget = Math.atan2(dy, dx);

        // Get motion profile reference velocity (mm/s)
        double vRef = profile.getTargetVelocity(profileTimer.seconds());

        // Feedforward + PID corrections (world frame)
        double vxFF = vRef * Math.cos(headingToTarget);
        double vyFF = vRef * Math.sin(headingToTarget);

        double vxPID = pidX.calculate(currentPose.getX(DistanceUnit.MM));
        double vyPID = pidY.calculate(currentPose.getY(DistanceUnit.MM));
        double vhPID = pidH.calculate(currentPose.getHeading(AngleUnit.DEGREES));

        double vxWorld = vxFF + vxPID * MAX_VEL_MM_S;
        double vyWorld = vyFF + vyPID * MAX_VEL_MM_S;

        // Convert to robot frame (field-centric)
        double headingRad = currentPose.getHeading(AngleUnit.RADIANS);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);
        double forward = vxWorld * cosH + vyWorld * sinH;
        double strafe  = -vxWorld * sinH + vyWorld * cosH;

        // Combine feedforward + PID + voltage compensation
        double ff = kV * vRef + kS * Math.signum(vRef);
        double voltageScale = getVoltageComp();
        double powerScale = ff * voltageScale;

        // Apply mecanum wheel powers
        setMecanumPowers(forward * powerScale, strafe * powerScale, vhPID * 0.015);

        // Check completion
        if (atTarget()) {
            stop();
        }
    }

    /** Stop all motion */
    public void stop() {
        robot.frontLeftMotor.setPower(0);
        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
        profileActive = false;
    }

    /** Returns true if within all tolerances and settled */
    public boolean atTarget() {
        boolean xyOK = pidX.atSetPoint() && pidY.atSetPoint();
        boolean hOK = pidH.atSetPoint();
        if (xyOK && hOK) {
            if (settleTimer.seconds() >= 0.3) return true;
        } else {
            settleTimer.reset();
        }
        return false;
    }

    /** Voltage compensation relative to 12V nominal */
    private double getVoltageComp() {
        double v = robot.getBatteryVoltageRobust();
        return (v > 6.0) ? (12.0 / v) : 1.0;
    }

    /** Mecanum drive mapping */
    private void setMecanumPowers(double forward, double strafe, double omega) {
        double lf = forward + strafe + omega;
        double rf = forward - strafe - omega;
        double lb = forward - strafe + omega;
        double rb = forward + strafe - omega;

        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));

        robot.frontLeftMotor.setPower(lf / max);
        robot.frontRightMotor.setPower(rf / max);
        robot.backLeftMotor.setPower(lb / max);
        robot.backRightMotor.setPower(rb / max);
    }

    // -------------------- Motion Profile Inner Class --------------------
    public static class MotionProfile {
        private final double distance, maxVel, maxAcc;
        private final double t1, t2, t3, vPeak;

        public MotionProfile(double distance, double maxVel, double maxAcc) {
            this.distance = distance;
            this.maxVel = maxVel;
            this.maxAcc = maxAcc;

            double tAccel = maxVel / maxAcc;
            double dAccel = 0.5 * maxAcc * tAccel * tAccel;

            if (2 * dAccel >= distance) {
                // triangular profile
                vPeak = Math.sqrt(distance * maxAcc);
                t1 = vPeak / maxAcc;
                t2 = t1;
                t3 = 2 * t1;
            } else {
                vPeak = maxVel;
                double dCruise = distance - 2 * dAccel;
                double tCruise = dCruise / maxVel;
                t1 = tAccel;
                t2 = t1 + tCruise;
                t3 = t2 + t1;
            }
        }

        public double getTargetVelocity(double t) {
            if (t <= t1) return maxAcc * t;
            else if (t <= t2) return vPeak;
            else if (t <= t3) return maxAcc * (t3 - t);
            else return 0;
        }
    }
}

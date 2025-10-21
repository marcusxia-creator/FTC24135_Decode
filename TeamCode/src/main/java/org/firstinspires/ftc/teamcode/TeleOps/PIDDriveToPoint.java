package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.ftc.MidpointTimer;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.TeleOps.RobotHardware;

public class PIDDriveToPoint {
    private RobotHardware robot = null;
    private final double ticksPerMM;
    private final double maxTicks;
    private final PIDController pidX;
    private final PIDController pidY;
    private final PIDController pidH;
    private MidpointTimer settleTimer;

    public enum DriveMotor{
        LEFT_FRONT,
        LEFT_BACK,
        RIGHT_FRONT,
        RIGHT_BACK
    }
    private enum Direction{
        x,
        y,
        h
    }
    private enum InBounds{
        NOT_IN_BOUNDS,
        IN_X_Y,
        IN_HEADING,
        IN_BOUNDS,
        DISABLE
    }

    private static double xyTolerance = 12;
    private static double headingTolerance = 0.5;
    private static double pGain = 0.008;

    private static double dGain = 0.00001;

    private static double accel = 10;

    private static double yawpGain = 5;
    private static double yawdGain = 0.0;
    private static double yawAccel = 20;

    private static double kv = 0.020;

    private static double ks = 0.120;

    private double leftFrontMotorOutput = 0;
    private double rightFrontMotorOutput = 0;
    private double leftBackMotorOutput   = 0;
    private double rightBackMotorOutput  = 0;

    private final ElapsedTime holdTimer = new ElapsedTime();
    private final ElapsedTime PIDTimer = new ElapsedTime();

    private boolean profileActive = false;

    private double power = 0.0;

    Pose2D targetPos2D;
    Pose2D currentPos2D;


    public PIDDriveToPoint(double ticksPerMM, double maxTicks, Pose2D targetPos2D, Pose2D currentPos2D) {
        this.ticksPerMM = ticksPerMM;
        this.maxTicks = maxTicks;

        this.targetPos2D = targetPos2D;
        this.currentPos2D = currentPos2D;

        this.pidX = new PIDController(pGain, 0, dGain);
        this.pidY = new PIDController(pGain, 0, dGain);
        this.pidH = new PIDController(yawpGain, 0, yawdGain);


        // Tolerance: normalized if maxTicks provided, else raw ticks
        if (maxTicks > 0) {
            pidX.setTolerance(10*ticksPerMM / maxTicks);
            pidY.setTolerance(10*ticksPerMM / maxTicks);
        } else {
            pidX.setTolerance(10*ticksPerMM);
            pidY.setTolerance(10*ticksPerMM);
        }

        // Tolerance for heading
        pidH.setTolerance(0.5);

        // Set target
        this.robot = robot;
    }

    public void setTarget(Pose2D targetPos2D) {
        this.targetPos2D = targetPos2D;

        pidX.setSetPoint(targetPos2D.getX(DistanceUnit.CM));
        pidX.setTolerance(0.5);
        pidY.setSetPoint(targetPos2D.getY(DistanceUnit.CM));
        pidY.setTolerance(0.5);
        pidH.setSetPoint(targetPos2D.getHeading(AngleUnit.DEGREES));
        pidH.setTolerance(1.0);
    }

    public void update(){
        double measurementX = currentPos2D.getX(DistanceUnit.CM) - targetPos2D.getX(DistanceUnit.CM);
        double measurementY = currentPos2D.getY(DistanceUnit.CM) - targetPos2D.getY(DistanceUnit.CM);
        double measurementH = currentPos2D.getHeading(AngleUnit.DEGREES) - targetPos2D.getHeading(AngleUnit.DEGREES);
        double setPointX = targetPos2D.getX(DistanceUnit.CM);
        double setPointY = targetPos2D.getY(DistanceUnit.CM);
        double setPointH = targetPos2D.getHeading(AngleUnit.DEGREES);
        double vx = pidX.calculate(measurementX,setPointX);
        double vy = pidX.calculate(measurementY,setPointY);
        double vh = pidX.calculate(measurementH,setPointH);
            }

    //Mecanum drive
    private void setMecanumPowers(double vx, double vy, double omega) {
        double lfPower = vx + vy + omega;
        double rfPower = vx - vy - omega;
        double lbPower = vx - vy + omega;
        double rbPower = vx + vy - omega;

        double max = Math.max(1.0, Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower),
                        Math.max(Math.abs(lbPower), Math.abs(rbPower)))));

        robot.frontLeftMotor.setPower(lfPower / max);
        robot.frontRightMotor.setPower(rfPower / max);
        robot.backLeftMotor.setPower(lbPower / max);
        robot.backRightMotor.setPower(rbPower / max);
    }

    //Helper - at target or not.
    public boolean atTarget() {
        // profile inactive AND PID within tolerance AND settled
        boolean pidOk = pidX.atSetPoint(); // make sure you set tolerance appropriately
        double settleHoldSec = 1;
        return !profileActive && pidOk && (settleTimer.seconds() >= settleHoldSec);
    }


    /// Convert linear inches to encoder ticks; adjust counts and diameter
    private double mmToTicks(double mm) {
        return mm * RobotActionConfig.TICKS_PER_MM_SLIDES;
    }

    public double getpower(){return power;}

    // call this each loop or cache periodically
    private double getVoltageComp() {
        double v = robot.getBatteryVoltageRobust(); // implement in your RobotHardware
        return (v > 6.0) ? (12.0 / v) : 1.0; // scale relative to 12V nominal
    }

    //Motion Profile
    public class MotionProfile2D {
        private double maxVel;
        private double maxAccel;

        public MotionProfile2D(double maxVel, double maxAccel) {
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
        }

        public double getTotalTime(double distance) {
            double tAccel = maxVel / maxAccel;
            double dAccel = 0.5 * maxAccel * tAccel * tAccel;
            double dCruise = distance - 2 * dAccel;
            double tCruise = Math.max(0, dCruise / maxVel);
            return 2 * tAccel + tCruise;
        }

        public double getTargetVelocity(double t, double distance) {
            double tAccel = maxVel / maxAccel;
            double dAccel = 0.5 * maxAccel * tAccel * tAccel;
            double dCruise = distance - 2 * dAccel;
            double tCruise = Math.max(0, dCruise / maxVel);
            double tTotal = 2 * tAccel + tCruise;

            if (t < tAccel) return maxAccel * t;
            else if (t < tAccel + tCruise) return maxVel;
            else if (t < tTotal) return maxAccel * (tTotal - t);
            else return 0.0;
        }
    }

}

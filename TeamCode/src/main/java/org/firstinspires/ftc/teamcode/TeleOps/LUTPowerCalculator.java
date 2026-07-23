package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

@Config
public class LUTPowerCalculator {
    // updateDistanceAndZone() is called from getPower(), getShooterAngle(), and
    // getDistance() — all invoked within the same loop tick. Guard against
    // recomputing the same pinpoint-derived distance/zone 2-3x per tick.
    /// use a shooterZone enum for clarity and dashboard tuning.
    public enum ShooterZone {
        ZONE_0(0),
        ZONE_1(1),
        ZONE_2(2),
        ZONE_3(3),
        ZONE_4(4),
        ZONE_5(5),
        ZONE_6(6),
        ZONE_7(7),
        ZONE_8(8);

        private final int number;

        ShooterZone(int number) {
            this.number = number;
        }

        public int getNumber() {
            return number;
        }
    }

    private final RobotHardware robot;
    // PID on normalized velocity (0..1)
    private final PIDController pid;

    public static final double tickToRPM = SHOOTER_RPM_CONVERSION;
    private final int maxVelocityRPM = shooterMaxRPM;

    private double distance;
    private ShooterZone  currentZone = ShooterZone.ZONE_0;
    private double shooterAdjusterAngle;
    private int rpmTarget;
    private double rpmMeasured;

    private final Pose2D redGoalPose  = new Pose2D(DistanceUnit.INCH, -70,  70, AngleUnit.DEGREES, -45);
    private final Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, -70, -70, AngleUnit.DEGREES,  45);
    private Pose2D actualGoalPose = redGoalPose;

    // --------- Tune these ----------
    // PID
    public static double kPShooter = 2;   // start ~1.5 to 4.0 (normalized units)
    public static double kIShooter = 0.0;
    public static double kDShooter = 0.02;

    // Feedforward
    public static double kSShooter = 0.03;  // static friction (small bump)
    public static double kVShooter = 1.0;  // scale from targetNorm to power (roughly 1.0 if perfect)
    // --------------------------------


    //===================================================
    //Constructor
    //===================================================
    public LUTPowerCalculator(RobotHardware robot) {
        this.robot = robot;
        this.pid = new PIDController(kPShooter, kIShooter, kDShooter);
    }

    public void setAlliance(boolean isRedAlliance) {
        actualGoalPose = (isRedAlliance) ? redGoalPose : blueGoalPose;
    }

    // Calculate the distance
    private void updateDistanceAndZone() {
        double dx = robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH);
        double dy = robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH);
        distance = Math.hypot(dx, dy);

        currentZone = calculateZone(distance);
    }

    // Determine the zone based on the distance
    private ShooterZone calculateZone(double distance) {
        if (!Double.isFinite(distance) || distance < 0.0) {
            return ShooterZone.ZONE_0;
        }
        if (distance > closeEdge && distance <= CLOSE) {
            return ShooterZone.ZONE_1;
        }
        if (distance > CLOSE && distance <= MID) {
            return ShooterZone.ZONE_2;
        }
        if (distance > MID && distance <= MidPoint) {
            return ShooterZone.ZONE_3;
        }
        if (distance > MidPoint && distance <= FAR) {
            return ShooterZone.ZONE_4;
        }
        if (distance > FAR && distance <= FAR_EDGE) {
            return ShooterZone.ZONE_5;
        }

        /*
         * Intentional distance gap:
         *
         * FAR_EDGE < distance <= FAR_ZONE_CLOSE
         *
         * returns ZONE_0.
         */
        if (distance > FAR_ZONE_TOUCH  && distance <= FAR_ZONE_CLOSE) {
            return ShooterZone.ZONE_6;
        }
        if (distance > FAR_ZONE_CLOSE  && distance <= FAR_ZONE_MID ) {
            return ShooterZone.ZONE_7;
        }
        /*
         * Replace these limits with your actual zone-8 limits.
         */
        if (distance > FAR_ZONE_MID  && distance <= FAR_ZONE_FAR) {
            return ShooterZone.ZONE_8;
        }
        return ShooterZone.ZONE_0;
    }

    // update the power based on the distance and zone
    // loop in FSMshooter for updates
    // getpower for compatiblity with FSMshooter
    public double getPower(){
        return getPower(true);
    }

    public double getPower(boolean enabled) {
        if (robot == null || robot.pinpoint == null || robot.topShooterMotor == null) return 0.0;

        //updatePIDParameters();// apply live PID changes from FTC dashboard

        updateState();// update distance and shooter Zone

        /*
         * Apply the Dashboard RPM adjustment factor.
         */
        double safeRPMFactor = Math.max(0.0, RPMFactor);
        rpmTarget = (int) Math.round(safeRPMFactor * rpmTarget );

        // If not shooting, return 0 and reset PID so it doesn't "wind up"
        if (!enabled || rpmTarget <= 0) {
            pid.reset();
            return 0.0;
        }
        // Measured RPM from encoder ticks/sec
        rpmMeasured = Math.abs(robot.topShooterMotor.getVelocity())* tickToRPM;

        // Voltage - aware max RPM
        double voltage = robot.getBatteryVoltageRobust();
        double maxRPMDynamic = maxVelocityRPM * voltage / REF_VOLTAGE;

        // Normalize to 0..1 for stable tuning
        double targetNorm = (double) rpmTarget / maxRPMDynamic;
        double currentNorm = (double) rpmMeasured / maxRPMDynamic;

        targetNorm = clamp01(targetNorm);
        currentNorm = clamp01(currentNorm);

        // Feedforward: baseline power
        double ff = (kSShooter * Math.signum(targetNorm)) + (kVShooter * targetNorm);

        // PID correction (FTCLib PIDController uses (measured, setpoint))
        double pidOut = pid.calculate(currentNorm, targetNorm);

        double output = ff + pidOut;

        if (Double.isNaN(output) || Double.isInfinite(output)) return 0.0;
        return clamp(output, -1.0, 1.0);
    }

    // updateState() is called from getPower() & Loop in FSMshooter
    private void updateState() {
        updateDistanceAndZone();
        rpmTarget = getTargetRpmForZone(currentZone);
        shooterAdjusterAngle = getShooterAngleForZone(currentZone);
    }

    private int getTargetRpmForZone(ShooterZone zone) {
        switch (zone) {
            case ZONE_8: return RPM8;
            case ZONE_7: return RPM7;
            case ZONE_6: return RPM6;
            case ZONE_5: return RPM5;
            case ZONE_4: return RPM4;
            case ZONE_3: return RPM3;
            case ZONE_2: return RPM2;
            case ZONE_1: return RPM1;
            case ZONE_0:
            default:
                return RPM0;
        }
    }
    private double getShooterAngleForZone(ShooterZone zone) {
        switch (zone) {
            case ZONE_1:
                return shooterAdjusterMin;
            case ZONE_2:
                return shooterAdjusterMid;
            case ZONE_3:
                return shooterAdjusterZone3;
            case ZONE_4:
            case ZONE_5:
            case ZONE_6:
            case ZONE_7:
            case ZONE_8:
            case ZONE_0:
            default:
                return shooterAdjusterMax;
        }
    }
    public int getZone(){
        return currentZone.getNumber();
    }

    public double getDistance(){
        return distance;
    }

    public int getRPMTarget() {
        return rpmTarget;
    }

    public double getMeasureRPM(){
        return rpmMeasured;
    }

    public double getShooterAngle() {
        return shooterAdjusterAngle;
    }

    private static double clamp01(double x) {
        return x < 0 ? 0 : (x > 1 ? 1 : x);
    }

    private static double clamp(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }
}

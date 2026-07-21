package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;


import java.util.Optional;
@Config
public class LUTPowerCalculator {

    private final RobotHardware robot;

    // PID on normalized velocity (0..1)
    private final PIDController pid;

    private double distance;
    private int zone = 0;

    // updateDistanceAndZone() is called from getPower(), getShooterAngle(), and
    // getDistance() — all invoked within the same loop tick. Guard against
    // recomputing the same pinpoint-derived distance/zone 2-3x per tick.
    private long lastZoneComputeNanos = -1;
    private static final long ZONE_RECOMPUTE_GUARD_NANOS = 1_000_000; // 1ms

    public static final double tickToRPM = SHOOTER_RPM_CONVERSION;
    private final int maxVelocityRPM = shooterMaxRPM;

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

    /**
     * LUT values goes from farest from goal to closes from goal
     * higher zone is further from goal
     */
    private final LUT<Integer, Integer> targetRPM = new LUT<Integer, Integer>() {{
        /// RPM is measured based on 13v
        add(8, RPM8); //4540
        add(7, RPM7); //4400
        add(6, RPM6); //4300
        add(5, RPM5); //3750
        add(4, RPM4); //3600
        add(3, RPM3); //3450
        add(2, RPM2); //3200
        add(1, RPM1); //3150
        add(0, RPM0); ///change this later
    }};

    //0.05
    //0.53
    private final LUT<Integer, Double> targetShootingAngle = new LUT<Integer, Double>() {{
        add(7, shooterAdjusterMax);
        add(6, shooterAdjusterMax);
        add(5, shooterAdjusterMax);
        add(4, shooterAdjusterMax);
        add(3, shooterAdjusterMax);
        add(2, shooterAdjusterMid);
        add(1, shooterAdjusterMin);
        add(0, shooterAdjusterMax);
    }};

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

    private void updateDistanceAndZone() {
        double dx = robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH);
        double dy = robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH);
        distance = Math.hypot(dx, dy);

        if (distance > closeEdge && distance <= CLOSE) zone     = 1;
        else if (distance > CLOSE && distance <= MID) zone      = 2;
        else if (distance > MID && distance <= MidPoint) zone   = 3;
        else if (distance > MidPoint && distance <= FAR) zone   = 4;
        else if (distance > FAR && distance <= FAR_EDGE) zone   = 5;
        else if (distance > FAR_ZONE_TOUCH && distance <= FAR_ZONE_CLOSE) zone = 6;
        else if (distance > FAR_ZONE_CLOSE && distance <= FAR_ZONE_MID) zone = 7;
        else if (distance > FAR_ZONE_MID && distance <= FAR_ZONE_FAR) zone = 8;
        else zone = 0;
    }

    public double getPower() {
        if (robot == null || robot.pinpoint == null || robot.topShooterMotor == null) return 0.0;
        updateDistanceAndZone();
        rpmTarget = Optional.ofNullable(targetRPM.get(zone)).orElse(0);
        rpmTarget=(int)Math.round(RPMFactor*rpmTarget);
        // If not shooting, return 0 and reset PID so it doesn't "wind up"
        if (rpmTarget <= 0) {
            pid.reset();
            return 0.0;
        }

        // Measured RPM from encoder ticks/sec
        rpmMeasured = robot.topShooterMotor.getVelocity() * tickToRPM;

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

    public int getZone(){
        return zone;
    }

    public double getDistance(){
        return distance;
    }

    public int getRPM() {
        return rpmTarget;
    }

    public double getMeasureRPM(){
        return rpmMeasured;
    }

    public double getShooterAngle() {
        ///int safeZone = Math.max(0, Math.min(zone, 3));
        return Optional.ofNullable(targetShootingAngle.get(zone)).orElse(shooterAdjusterMax);
    }

    private static double clamp01(double x) {
        return x < 0 ? 0 : (x > 1 ? 1 : x);
    }

    private static double clamp(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }
}

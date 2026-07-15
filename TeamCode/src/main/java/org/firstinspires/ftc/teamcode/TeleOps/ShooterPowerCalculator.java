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
public class ShooterPowerCalculator {

    private final RobotHardware robot;
    // PID on normalized velocity (0..1)
    private final PIDController pid;

    private double distance;
    private int currentZone = 0;

    private static final double tickToRPM = SHOOTER_RPM_CONVERSION;
    private final int maxVelocityRPM = shooterMaxRPM;

    private int rpmTarget;
    private double rpmMeasured;
    private double shooterAdjusterAngle;

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
        add(7, RPM7);
        add(6, RPM6); // 5200
        add(5, RPM5); // 4450
        add(4, RPM4); // 4440
        add(3, RPM3); // 4470
        add(2, RPM2); // 3890
        add(1, RPM1); // 3750
        add(0, RPM0); ///change this later
    }};

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
    public ShooterPowerCalculator(RobotHardware robot) {
        this.robot = robot;
        this.pid = new PIDController(kPShooter, kIShooter, kDShooter);
    }

    /** * Selects the goal used for distance calculation.
      * * @param isRedAlliance true for red alliance; false for blue alliance
     * */
    public void setAlliance(boolean isRedAlliance) {
        actualGoalPose = (isRedAlliance) ? redGoalPose : blueGoalPose;
    }

    /**
     * * Calculates shooter motor power.
     * * Call this once during each OpMode loop.
     * * @return motor power from 0.0 to 1.0
     */
    public double getPower() {
        if (robot == null || robot.pinpoint == null || robot.topShooterMotor == null) return 0.0;

        /// update distance, zone, and target RPM every loop.
        updateState();

        /// If not shooting, return 0 and reset PID so it doesn't "wind up"
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

    /** * Updates distance, zone. */
    private void updateDistanceAndZone() {
        double dx = robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH);
        double dy = robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH);
        distance = Math.hypot(dx, dy);
        currentZone = calculateZone(distance);
    }

    /** * update distance, zone, and target RPM. */
    private void updateState() {
        updateDistanceAndZone();
        rpmTarget = Optional.ofNullable(targetRPM.get(currentZone)).orElse(0);
        shooterAdjusterAngle = getShooterAngleForZone(currentZone);
    }

    private int calculateZone(double distance) {
        if (!Double.isFinite(distance) || distance < 0.0) {
            return 0;
        }
        if (distance > closeEdge && distance <= CLOSE) {
            return 1;
        }
        if (distance > CLOSE && distance <= MID) {
            return 2;
        }
        if (distance > MID && distance <= MidPoint) {
            return 3;
        }
        if (distance > MidPoint && distance <= FAR) {
            return 4;
        }
        if (distance > FAR && distance <= FAR_EDGE) {
            return 5;
        }
        // Intentional gap:
        // FAR_EDGE < distance <= FAR_ZONE_CLOSE
        // returns zone 0.
        if (distance > FAR_ZONE_CLOSE
                && distance <= FAR_ZONE_MID) {
            return 6;
        }
        if (distance > FAR_ZONE_MID
                && distance <= FAR_ZONE_FAR) {
            return 7;
        }
        return 0;
    }

    /**
     * find RPM and shooting angle based on distance
     * @return
     */
    /** * Returns the target RPM for the supplied zone. */
    private int getTargetRpmForZone(int zone) {
        switch (zone)
        { case 1: return RPM1;
            case 2: return RPM2;
            case 3: return RPM3;
            case 4: return RPM4;
            case 5: return RPM5;
            case 6: return RPM6;
            case 7: return RPM7;
            case 0:
                default: return RPM0; } }
    /** * Returns the shooter adjuster position for the supplied zone. */
    private double getShooterAngleForZone(int zone)
        { switch (zone)
        {  case 1: return shooterAdjusterMin;
            case 2: return shooterAdjusterMid;
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 0:
            default: return shooterAdjusterMax; }
        }
   /// Getter Helper
    public int getCurrentZone(){
        return currentZone;
    }

    public double getDistance(){
        updateDistanceAndZone();
        return distance;
    }

    public int getRPM() {
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

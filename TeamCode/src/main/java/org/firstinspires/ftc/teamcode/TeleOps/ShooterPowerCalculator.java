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

    private double activeKP = Double.NaN;
    private double activeKI = Double.NaN;
    private double activeKD = Double.NaN;
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


    //===================================================
    //Constructor
    //===================================================
    public ShooterPowerCalculator(RobotHardware robot) {
        this.robot = robot;
        this.pid = new PIDController(kPShooter, kIShooter, kDShooter);
        activeKP = kPShooter;
        activeKI = kIShooter;
        activeKD = kDShooter;
    }

    /** * Selects the goal used for distance calculation.
      * * @param isRedAlliance true for red alliance; false for blue alliance
     * */
    public void setAlliance(boolean isRedAlliance) {
        actualGoalPose = (isRedAlliance) ? redGoalPose : blueGoalPose;
    }

    /**
     * Compatibility method.
     *
     * Calling getPower() means shooter control is enabled.
     */
    public double getPower() {
        return getPower(true);
    }
    /**
     * * Calculates shooter motor power.
     * * Call this once during each OpMode loop.
     * * @return motor power from 0.0 to 1.0
     */
    public double getPower(boolean enabled) {
        if (robot == null || robot.pinpoint == null || robot.topShooterMotor == null) return 0.0;

        // Apply changes made through FTC Dashboard.
        updatePIDParameters();

        /// update distance, zone, and target RPM every loop.
        updateState();

        /*
         * Always update measured RPM so telemetry and the FSM can
         * see the actual flywheel speed, including during coast-down.
         */
        double shooterTicksPerSecond =
                Math.abs(
                        robot.topShooterMotor.getVelocity()
                );

        rpmMeasured =
                shooterTicksPerSecond
                        * SHOOTER_RPM_CONVERSION;

        /*
         * Do not allow PID accumulation while the flywheel is off.
         */
        if (!enabled || rpmTarget <= 0) {
            pid.reset();
            return 0.0;
        }

        // Voltage - aware max RPM
        double voltage = robot.getBatteryVoltageRobust();
        double maxRPMDynamic = shooterMaxRPM * voltage / REF_VOLTAGE; //voltage corrected max RPM
        // Protect against invalid configuration or voltage.
        if (!Double.isFinite(maxRPMDynamic)
                || maxRPMDynamic <= 0.0) {
            pid.reset();
            return 0.0;
        }

        // Normalize to 0..1 for stable tuning
        double targetNorm = (double) rpmTarget / maxRPMDynamic;
        double currentNorm = (double) rpmMeasured / maxRPMDynamic;

        targetNorm = clamp01(targetNorm);
        currentNorm = clamp01(currentNorm);

        double staticFeedforward =
                targetNorm > 0.005
                        ? kSShooter
                        : 0.0;

        // Feedforward: baseline power
        double ff = staticFeedforward + (kVShooter * targetNorm);

        // PID correction (FTCLib PIDController uses (measured, setpoint))
        double pidOut = pid.calculate(currentNorm, targetNorm);

        double output = ff + pidOut;

        if (Double.isNaN(output) || Double.isInfinite(output)) return 0.0;
        return clamp(output, 0.0, 1.0);
    }

    /** * update distance, zone, and target RPM. */
    private void updateState() {
        updateDistanceAndZone();

        rpmTarget =
                getTargetRpmForZone(currentZone);

        shooterAdjusterAngle =
                getShooterAngleForZone(currentZone);
    }

    /** * Updates distance, zone. */
    private void updateDistanceAndZone() {
        double dx = robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH);
        double dy = robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH);
        distance = Math.hypot(dx, dy);
        currentZone = calculateZone(distance);
    }

    private int calculateZone(double distance) {
        if (!Double.isFinite(distance)) {
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
                default: return RPM0; }
    }
    /** * Returns the shooter adjuster position for the supplied zone. */
    private double getShooterAngleForZone(int zone)
        { switch (zone)
        {  case 1: return shooterAdjusterMin;
            case 2: return shooterAdjusterMid;
            case 3: return shooterAdjusterMid;
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

    private void updatePIDParameters() {
        boolean pidChanged =
                Double.compare(activeKP, kPShooter) != 0
                        || Double.compare(activeKI, kIShooter) != 0
                        || Double.compare(activeKD, kDShooter) != 0;

        if (!pidChanged) {
            return;
        }

        pid.setPID(
                kPShooter,
                kIShooter,
                kDShooter
        );

        pid.reset();

        activeKP = kPShooter;
        activeKI = kIShooter;
        activeKD = kDShooter;
    }
}

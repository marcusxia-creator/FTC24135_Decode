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

    public static final double tickToRPM = SHOOTER_RPM_CONVERSION;
    private final int maxVelocityRPM = shooterMaxRPM;

    private int rpmTarget;
    private double rpmMeasured;

    private final Pose2D redGoalPose  = new Pose2D(DistanceUnit.INCH, -70,  70, AngleUnit.DEGREES, -45);
    private final Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, -70, -70, AngleUnit.DEGREES,  45);
    private Pose2D actualGoalPose = redGoalPose;

    // --------- Tune these ----------
    // PID
    public static double kP = 2;   // start ~1.5 to 4.0 (normalized units)
    public static double kI = 0.0;
    public static double kD = 0.02;

    // Feedforward
    public static double kS = 0.03;  // static friction (small bump)
    public static double kV = 1.0;  // scale from targetNorm to power (roughly 1.0 if perfect)
    // --------------------------------

    /**
     * LUT values goes from farest from goal to closes from goal
     * higher zone is further from goal
     */
    private final LUT<Integer, Integer> targetRPM = new LUT<Integer, Integer>() {{
        /// RPM is measured based on 13v
        add(6, RPM6); // 5200
        add(5, RPM5); // 4450
        add(4, RPM4); // 4440
        add(3, RPM3); // 4470
        add(2, RPM2); // 3890
        add(1, RPM1); // 3750
        add(0, RPM0); ///change this later
    }};

    //0.05
    //0.53
    private final LUT<Integer, Double> targetShootingAngle = new LUT<Integer, Double>() {{
        add(6, shooterAdjusterMax);
        add(5, shooterAdjusterMax);
        add(4, shooterAdjusterMax);
        add(3, shooterAdjusterMax);
        add(2, shooterAdjusterMid);
        add(1, shooterAdjusterMin);
        add(0, shooterAdjusterMax);
    }};

    public LUTPowerCalculator(RobotHardware robot) {
        this.robot = robot;
        this.pid = new PIDController(kP, kI, kD);
    }

    public void setAlliance(boolean isRedAlliance) {
        actualGoalPose = isRedAlliance ? redGoalPose : blueGoalPose;
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
        else if (distance > FAR_ZONE_LOW && distance <= FAR_ZONE_HIGH) zone = 6;
        else zone = 0;
    }

    public int getRPM() {
        updateDistanceAndZone();
        rpmTarget = Optional.ofNullable(targetRPM.get(zone)).orElse(0);
        return rpmTarget;
    }

    public double getPower() {
        if (robot == null || robot.pinpoint == null || robot.topShooterMotor == null) return 0.0;
        updateDistanceAndZone();
        int rpmTarget = Optional.ofNullable(targetRPM.get(zone)).orElse(0);
        // If not shooting, return 0 and reset PID so it doesn't "wind up"
        if (rpmTarget <= 0) {
            pid.reset();
            return 0.0;
        }

        // Measured RPM from encoder ticks/sec
        rpmMeasured = robot.topShooterMotor.getVelocity() * tickToRPM;

        // Normalize to 0..1 for stable tuning
        double targetNorm = rpmTarget / (double) maxVelocityRPM;
        double currentNorm = rpmMeasured / (double) maxVelocityRPM;

        targetNorm = clamp01(targetNorm);
        currentNorm = clamp01(currentNorm);

        // Feedforward: baseline power
        double ff = (kS * Math.signum(targetNorm)) + (kV * targetNorm);

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
        updateDistanceAndZone();
        return distance;
    }

    public double getMeasureRPM(){
        return rpmMeasured;
    }

    public double getShooterAngle() {
        ///int safeZone = Math.max(0, Math.min(zone, 3));
        updateDistanceAndZone();
        return Optional.ofNullable(targetShootingAngle.get(zone)).orElse(shooterAdjusterMax);
    }

    private static double clamp01(double x) {
        return x < 0 ? 0 : (x > 1 ? 1 : x);
    }

    private static double clamp(double x, double low, double high) {
        return Math.max(low, Math.min(high, x));
    }
}

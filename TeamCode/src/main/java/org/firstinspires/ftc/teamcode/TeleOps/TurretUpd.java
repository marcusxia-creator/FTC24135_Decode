package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class TurretUpd {
    /*
      Revamp goals:
      1) Turret class does NOT talk to Limelight directly (no robot.limelight calls here)
      2) Turret exposes clean APIs:
         - setAimMode(AimMode)
         - setFieldAimPoint(x,y) or setTargetFieldAngleDeg()
         - updateVisionTx(txDeg) (optional)
         - update() to apply PID and drive motor
      3) Vision correction is treated as an "offset" term (filtered + clamped)
      4) Wrap angles properly and avoid mixing degrees/ticks
      5) Dashboard PID updates remain supported
     */

    private final RobotHardware robot;

    // ---------------------------
    // Encoder conversion
    // ---------------------------
    private final double tickToAngleDeg = ((0.16867469879518 * 360.0) / 145.1);
    private final double angleDegToTick = 1.0 / tickToAngleDeg;

    // ---------------------------
    // Dashboard tuning
    // ---------------------------
    public static double kP = 17, kI = 0, kD = 0.005;
    public static double kS = 0.0;     // static feedforward (optional)
    public static double kV = 0.0;     // velocity/feedforward (optional) -- set 0 if unsure

    public static double kP_motor = 17, kI_motor = 0, kD_motor = 0.005, kF = 2;

    // ---------------------------
    // Motion limits / safety
    // ---------------------------
    public static double minTurretDeg = -170;
    public static double maxTurretDeg = 170;
    public static double maxPower = 1.0;

    // ---------------------------
    // Vision assist parameters
    // ---------------------------
    public static boolean visionAssistEnabled = true;
    public static double visionDeadbandDeg = 0.5;     // ignore tiny tx jitter
    public static double visionBlend = 0.35;          // 0..1, how quickly tx accumulates into offset
    public static double visionOffsetClampDeg = 30.0; // prevent runaway offsets
    public static boolean visionInvertTx = false;     // flip if sign is wrong on your bot

    // ---------------------------
    // Aim modes
    // ---------------------------
    public enum AimMode {
        FIELD_POINT,      // aim at a fixed (x,y) point on the field
        FIELD_ANGLE,      // aim at a specified field angle (deg)
        HOLD_CURRENT      // hold the turret where it is
    }

    private AimMode aimMode = AimMode.FIELD_POINT;

    // FIELD_POINT target (inches)
    private double targetFieldXIn = -66;  // default example
    private double targetFieldYIn =  66;

    // FIELD_ANGLE target (deg)
    private double targetFieldAngleDeg = 0.0;

    // Vision correction offset (deg) added to the geometric target
    private double visionOffsetDeg = 0.0;

    // PID
    private final PIDController pidController;

    // Motor PIDF (RUN_USING_ENCODER)
    private PIDFCoefficients pidf = new PIDFCoefficients(kP_motor, kI_motor, kD_motor, kF);

    // Track last dashboard values
    private double lastkP = Double.NaN, lastkI = Double.NaN, lastkD = Double.NaN;
    private double lastkPmotor = Double.NaN, lastkImotor = Double.NaN, lastkDmotor = Double.NaN, lastkF = Double.NaN;

    public TurretUpd(RobotHardware robot) {
        this.robot = robot;
        this.pidController = new PIDController(kP, kI, kD);
        this.robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    // =========================================================
    // Public API: configuration / targets
    // =========================================================

    public void setAimMode(AimMode mode) {
        if (mode != this.aimMode) {
            this.aimMode = mode;
            // When changing modes, you usually want to clear vision offset
            // so you don't carry old tx into a new target.
            resetVisionAssist();
        }
    }

    /** Aim at a fixed field point (inches), using pinpoint pose (inches). */
    public void setFieldAimPoint(double xIn, double yIn) {
        targetFieldXIn = xIn;
        targetFieldYIn = yIn;
        setAimMode(AimMode.FIELD_POINT);
    }

    /** Aim at a field-referenced angle (deg). */
    public void setTargetFieldAngleDeg(double angleDeg) {
        targetFieldAngleDeg = wrapDeg(angleDeg);
        setAimMode(AimMode.FIELD_ANGLE);
    }

    /** Hold turret at its current mechanical angle. */
    public void holdCurrent() {
        // Convert current turret angle (robot frame) to an equivalent field angle target.
        // In HOLD_CURRENT we command the motor to hold its current ticks directly.
        setAimMode(AimMode.HOLD_CURRENT);
    }

    // =========================================================
    // Vision assist: input tx from main loop / FSM
    // =========================================================

    /** Call from main loop when Limelight is valid. tx is degrees. */
    public void updateVisionTx(double txDeg) {
        if (!visionAssistEnabled) return;

        if (visionInvertTx) txDeg = -txDeg;

        // deadband
        if (Math.abs(txDeg) < visionDeadbandDeg) txDeg = 0.0;

        // blend into offset (smooth, avoids snapping)
        visionOffsetDeg += visionBlend * txDeg;

        // clamp
        visionOffsetDeg = Range.clip(visionOffsetDeg, -visionOffsetClampDeg, visionOffsetClampDeg);
    }

    /** Call from main loop when Limelight is invalid or you want to disable assist right now. */
    public void resetVisionAssist() {
        visionOffsetDeg = 0.0;
    }

    public double getVisionOffsetDeg() {
        return visionOffsetDeg;
    }

    // =========================================================
    // Main update: call every loop
    // =========================================================

    /** Call every loop: updates PID params and drives the motor. */
    public void update() {
        updatePidFromDashboard();

        switch (aimMode) {
            case HOLD_CURRENT:
                // simplest hold: RUN_TO_POSITION at current ticks or PID hold
                holdAtCurrentTicks();
                break;

            case FIELD_ANGLE:
            case FIELD_POINT:
            default:
                driveTurretPID();
                break;
        }
    }

    // =========================================================
    // Core computations
    // =========================================================

    /** Turret motor angle in degrees (mechanical), from encoder. */
    public double getTurretMotorAngleDeg() {
        return robot.turretMotor.getCurrentPosition() * tickToAngleDeg;
    }

    /** Robot heading in degrees (field frame). */
    public double getRobotHeadingDeg() {
        return robot.pinpoint.getHeading(AngleUnit.DEGREES);
    }

    /** Desired field angle (deg) based on aim mode (without vision). */
    public double getBaseTargetFieldAngleDeg() {
        if (aimMode == AimMode.FIELD_ANGLE) {
            return targetFieldAngleDeg;
        }
        // FIELD_POINT
        return Math.toDegrees(Math.atan2(
                (targetFieldYIn - robot.pinpoint.getPosY(DistanceUnit.INCH)),
                (targetFieldXIn - robot.pinpoint.getPosX(DistanceUnit.INCH))
        ));
    }

    /**
     * Final target field angle (deg) including optional vision offset.
     * Vision offset is applied in field-angle space because your tx is a yaw error.
     */
    public double getFinalTargetFieldAngleDeg() {
        double base = getBaseTargetFieldAngleDeg();
        return wrapDeg(base + (visionAssistEnabled ? visionOffsetDeg : 0.0));
    }

    /**
     * Turret drive angle (deg) in robot/turret frame:
     * error = robotHeading - targetFieldAngle
     * wrapped to [-180,180], then sign-adjusted to match your original behavior.
     */
    public double getTurretDriveAngleDeg() {
        double target = getFinalTargetFieldAngleDeg();
        double error = wrapDeg(getRobotHeadingDeg() - target);
        return -error; // keep your original sign convention
    }

    /** Target ticks based on turret drive angle. */
    public int getTargetTicks() {
        double clippedDeg = Range.clip(getTurretDriveAngleDeg(), minTurretDeg, maxTurretDeg);
        return (int) Math.round(clippedDeg * angleDegToTick);
    }

    public int getCurrentTicks() {
        return robot.turretMotor.getCurrentPosition();
    }

    // =========================================================
    // Motor driving
    // =========================================================

    /** Simple RUN_TO_POSITION drive (works, but PID loop gives nicer behavior). */
    public void driveTurretMotor() {
        int ticks = getTargetTicks();
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(maxPower);
    }

    /** Your FTCLib PID on ticks + optional FF. */
    public void driveTurretPID() {
        int targetTicks = getTargetTicks();
        int currentTicks = getCurrentTicks();

        // Feedforward: keep conservative unless you truly characterized it
        double ff = 0.0;
        if (kS != 0.0) ff += kS * Math.signum(targetTicks - currentTicks);

        // kV in ticks-space is usually not what you want; leave 0 unless tuned carefully
        // If you want velocity FF, compute from desired velocity, not absolute ticks.
        if (kV != 0.0) {
            // WARNING: this is placeholder behavior; prefer velocity-based FF if you use kV.
            ff += kV * 0.0;
        }

        double pid = pidController.calculate(currentTicks, targetTicks);
        double output = Range.clip(pid + ff, -maxPower, maxPower);

        robot.turretMotor.setPower(output);
    }

    private void holdAtCurrentTicks() {
        // Hold current position using RUN_TO_POSITION at current tick
        int hold = getCurrentTicks();
        robot.turretMotor.setTargetPosition(hold);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(maxPower);
    }

    // =========================================================
    // Dashboard PID updates
    // =========================================================

    public void updatePidFromDashboard() {
        if (kP != lastkP || kI != lastkI || kD != lastkD) {
            pidController.setPID(kP, kI, kD);
            lastkP = kP; lastkI = kI; lastkD = kD;
        }

        if (kP_motor != lastkPmotor || kI_motor != lastkImotor || kD_motor != lastkDmotor || kF != lastkF) {
            pidf = new PIDFCoefficients(kP_motor, kI_motor, kD_motor, kF);
            robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastkPmotor = kP_motor; lastkImotor = kI_motor; lastkDmotor = kD_motor; lastkF = kF;
        }
    }

    // =========================================================
    // Utilities
    // =========================================================

    /** Wrap degrees to [-180, 180). */
    private static double wrapDeg(double deg) {
        deg = deg % 360.0;
        if (deg >= 180.0) deg -= 360.0;
        if (deg < -180.0) deg += 360.0;
        return deg;
    }
}

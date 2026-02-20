package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueFarGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redFarGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_X_Offset;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.turret_Center_Y_Offset;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Optional;

@Config
public class Turret {
    /*We will control the turret using input from the pinpoint
    turret is continuously running and is separate from the shooter control
     */

    private final RobotHardware robot;

    private final double tickToAngle = ((0.16867469879518 * 360) / 145.1);  //1 tick = 0.42 deg
    private final double angleToTick = 1.0 / tickToAngle;                   //2.389 tick = 1deg
    private double conversionFactor = 39.3700787;

    public static double kPTurret = 0.004, kITurret = 0, kDTurret = 0.0003, kSTurret = 0.0002, kVTurret = 0.004;     // turret motor drive pidcontroller
    public static double kP_motor = 20, kI_motor = 0, kD_motor = 0.005, kF = 2; // turret motor pidf
    private final double THETA = Math.atan(turret_Center_Y_Offset / turret_Center_X_Offset);

    private final LUT<Integer, Pose2D> redTargetPose = new LUT<Integer, Pose2D>() {{
        add(1, redCloseGoalPose);
        add(2, redFarGoalPose);
    }};

    private final LUT<Integer, Pose2D> blueTargetPose = new LUT<Integer, Pose2D>() {{
        add(1, blueCloseGoalPose);
        add(2, blueFarGoalPose);
    }};

    private LUT<Integer, Pose2D> targetPose;

    private Pose2D goalPose;

    private PIDController pidController;
    private Limelight limelight;

    PIDFCoefficients pidf = new PIDFCoefficients(
            kP_motor,      // P
            kI_motor,      // I
            kD_motor,      // D
            kF               // F
    );
    private double lastkP = Double.NaN, lastkI = Double.NaN, lastkD = Double.NaN;
    private double lastkPmotor = Double.NaN, lastkImotor = Double.NaN, lastkDmotor = Double.NaN, lastkF = Double.NaN;

    // NEW (does not rename anything): prevents mode spam/jumpy target angle
    private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);
    private boolean runToPositionConfigured = false;
    private double filteredTargetAngle = 0;
    private boolean firstAngleUpdate = true;
    private double alpha = 0.2;



    //--------------------------------------------
    // Constructor
    //--------------------------------------------
    public Turret (RobotHardware robot, boolean isRedAlliance) {
        this.robot = robot;
        pidController = new PIDController(kPTurret, kITurret, kDTurret);

        // Set motor PIDF once using your existing pidf field
        this.robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // KEEP logic but simplify
        targetPose = isRedAlliance ? redTargetPose : blueTargetPose;

        // Prevent goalPose null before updateZoneForGoalPose() is called
        goalPose = Optional.ofNullable(targetPose.get(1)).orElse(redCloseGoalPose);

        // TODO : Optional: ready the turret immediately
        //initTurret();
    }

    public void initTurret() {
        robot.turretMotor.setTargetPosition(0);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
        runToPositionConfigured = true;
    }

    public void resetTurretPosition() {
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runToPositionConfigured = false;
    }

    public int motorDriveTick() {
        return (int) Math.round(getTurretDriveAngle() * angleToTick);
    }

    public void updatePidFromDashboard() {
        // FTCLib PID (your own controller)
        if (kPTurret != lastkP || kITurret != lastkI || kDTurret != lastkD) {
            pidController.setPID(kPTurret, kITurret, kDTurret);
            lastkP = kPTurret; lastkI = kITurret; lastkD = kDTurret;
        }

        // Motor controller PIDF (RUN_USING_ENCODER) — only if you really need it
        if (kP_motor != lastkPmotor || kI_motor != lastkImotor || kD_motor != lastkDmotor || kF != lastkF) {
            pidf = new PIDFCoefficients(kP_motor, kI_motor, kD_motor, kF);
            robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

            lastkPmotor = kP_motor; lastkImotor = kI_motor; lastkDmotor = kD_motor; lastkF = kF;
        }
    }

    public double getTurretDriveAngle () {
        return -(floorMod(robot.pinpoint.getHeading(AngleUnit.DEGREES) - getTargetAngle()+180, 360)-180);
    }

    public double getTurretMotorAngle(){
        return (robot.turretMotor.getCurrentPosition() * tickToAngle);
    }

    public void driveTurretMotor(){
        /** OLD METHOD*/
        //updatePidFromDashboard();
        if (!runToPositionConfigured || robot.turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            runToPositionConfigured = true;
        }
        int ticks = (int) Math.round(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
    }

    public void driveTurretPID() {
        //updatePidFromDashboard();

        int targetTicks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        int currentTicks = robot.turretMotor.getCurrentPosition();
        int errorTicks = targetTicks - currentTicks;

        double power = pidController.calculate(currentTicks, targetTicks);
        // NEW -- Turn error into a bounded "move command" [-1..1] using tanh
        double errGain = 1.0 / (90*angleToTick);              // ~1 at ~300 ticks error (tune)
        double velCmd  = Math.tanh(errGain * errorTicks); // [-1..1]

        // NEW -- Feedforward should NOT be based on absolute targetTicks (too large).
        // Use direction + error assist.
        double ff =0.0;
        if (Math.abs(errorTicks)>5){
            ff = (kSTurret * Math.signum(errorTicks)) + (kVTurret * velCmd);
        }
        double output = power + ff;
        // NEW -- Soft-limit (smooth saturation) instead of hard clip
        double maxPower = 0.95;                     // turret safety cap (tune)
        double satGain  = 2.0;                     // higher = saturates sooner
        double cmd = maxPower * Math.tanh(satGain * output);
        robot.turretMotor.setPower(Range.clip(cmd, -1.0, 1.0));
    }

    public int getTargetTick () {
        return (int)Math.round(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
    }

    public int getCurrentTick () {
        return robot.turretMotor.getCurrentPosition();
    }

    public double getTargetAngle () {

        // NEW -- Fail-safe: if goalPose somehow isn't set, don't spin
        if (goalPose == null) return robot.pinpoint.getHeading(AngleUnit.DEGREES);
        double turretYaw = THETA + robot.pinpoint.getHeading(AngleUnit.RADIANS);

        double turretYOffSet = Math.sin(turretYaw) * (turretCenterOffsetLength * conversionFactor);
        double turretXOffSet = Math.cos(turretYaw) * (turretCenterOffsetLength * conversionFactor);

        double turretcentX = robot.pinpoint.getPosX(DistanceUnit.INCH) + turretXOffSet;
        double turretcentY = robot.pinpoint.getPosY(DistanceUnit.INCH) + turretYOffSet;

        //FIXME OLD
        //return Math.toDegrees(Math.atan2((goalPose.getY(DistanceUnit.INCH)-turretcentY), (goalPose.getX(DistanceUnit.INCH)-turretcentX)));
        //return Math.toDegrees(Math.atan2((goalPose.getY(DistanceUnit.INCH)-robot.pinpoint.getPosY(DistanceUnit.INCH)), (goalPose.getX(DistanceUnit.INCH)-robot.pinpoint.getPosX(DistanceUnit.INCH))));

        //New
        double rawAngle = Math.toDegrees(
                Math.atan2(
                        (goalPose.getY(DistanceUnit.INCH) - turretcentY),
                        (goalPose.getX(DistanceUnit.INCH) - turretcentX)
                )
        );

        // ---- Normalize to [-180, 180] ----
        rawAngle = AngleUnit.normalizeDegrees(rawAngle);

        // ---- First run initialization ----
        if (firstAngleUpdate) {
            filteredTargetAngle  = rawAngle;
            firstAngleUpdate = false;
            return rawAngle;
        }

        // ---- Compute shortest angular difference ----
        double delta = AngleUnit.normalizeDegrees(rawAngle - filteredTargetAngle );

        // ---- Deadband tolerance (2 degrees) ----
        if (Math.abs(delta) < 1.0) {
            return filteredTargetAngle;   // ignore jitter
        }

        // Exponential smoothing (0.2–0.3 recommended)
        if (Math.abs(delta) > 8) {
            alpha = 0.8;      // moving fast → respond fast
        }
        else if (Math.abs(delta) > 3) {
            alpha = 0.5;      // medium movement
        }
        else {
            alpha = 0.2;      // small jitter → heavy smoothing
        }

        filteredTargetAngle =
                AngleUnit.normalizeDegrees(
                        filteredTargetAngle + alpha * delta
                );

        return filteredTargetAngle;
    }

    public void updateZoneForGoalPose(int zone) {
        int normalizedZone;

        if (zone <= 5) {
            normalizedZone = 1;
        }
        else {
            normalizedZone = 2;
        }

        goalPose = Optional.ofNullable(targetPose.get(normalizedZone)).orElse(targetPose.get(1));
    }

    public Pose2D getGoalPose() {
        return goalPose;
    }

    private double floorMod(double x, double y){
        return x-(Math.floor(x/y) * y);
    }
}
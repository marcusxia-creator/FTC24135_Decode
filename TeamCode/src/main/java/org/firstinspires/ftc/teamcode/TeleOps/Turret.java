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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.filter.KalmanFilter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.security.cert.PKIXRevocationChecker;
import java.util.Optional;

@Config
public class Turret {
    /*We will control the turret using input from the pinpoint
    turret is continuously running and is separate from the shooter control
     */

    private final RobotHardware robot;

    public static double txToTickMultiplier = 10;
    private final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    private final double angleToTick = 1.0 / tickToAngle;
    private double conversionFactor = 39.3700787;

    //0.00001 0 0.005 0.2 2
    //kp 0.004
    //ks 0.0001
    //kv 0.005
    public static double kP = 0.001, kI = 0, kD = 0.0003, kS = 0.0001, kV = 0.002; // turret motor drive pidcontroller
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


    private final double turretCenterOffsetLength = Math.hypot(turret_Center_Y_Offset, turret_Center_X_Offset);

    // NEW (does not rename anything): prevents mode spam
    private boolean runToPositionConfigured = false;

    /**
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
     **/


    public Turret (RobotHardware robot, boolean isRedAlliance) {
        this.robot = robot;
        pidController = new PIDController(kP, kI, kD);

        // Set motor PIDF once using your existing pidf field
        ///this.robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // KEEP logic but simplify
        targetPose = isRedAlliance ? redTargetPose : blueTargetPose;

        // Prevent goalPose null before updateZoneForGoalPose() is called
        goalPose = Optional.ofNullable(targetPose.get(1)).orElse(redCloseGoalPose);

        // TODO : Optional: ready the turret immediately
        //initTurret();
    }

    public int motorDriveTick() {
        return (int) Math.round(getTurretDriveAngle() * angleToTick);
    }

    public void updatePidFromDashboard() {
        // FTCLib PID (your own controller)
        if (kP != lastkP || kI != lastkI || kD != lastkD) {
            pidController.setPID(kP, kI, kD);
            lastkP = kP; lastkI = kI; lastkD = kD;
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
        /** OLD METHOD
        //updatePidFromDashboard();
        int ticks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
        */

        // NEW! Only configure RUN_TO_POSITION once (instead of every loop)
        if (!runToPositionConfigured || robot.turretMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {

            runToPositionConfigured = true;
        }
        int ticks = (int) Math.round(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
    }

    public void driveTurretPID(int currentTick, int targetTick) {
        //updatePidFromDashboard();

        int errorTicks = targetTick - currentTick;
        // Feedforward should NOT be based on absolute targetTicks (too large).
        // Use direction + error assist.
        double ff = (kS * Math.signum(errorTicks)) + (kV * errorTicks);
        double power = pidController.calculate(currentTick, targetTick);
        double output = power + ff;
        robot.turretMotor.setPower(Range.clip(output, -1.0, 1.0));
    }

    public void driveTurretLimelight() {
        LLResult llResult = robot.limelight.getLatestResult();
        int targetTicks;
        int currentTicks = robot.turretMotor.getCurrentPosition();
        if (llResult != null && llResult.isValid()) {
            targetTicks = (int) (llResult.getTx() * txToTickMultiplier);
        }
        else {
            targetTicks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        }

        int errorTicks = targetTicks - currentTicks;
        double ff = (kS * Math.signum(errorTicks) + (kV * errorTicks));
        double power = pidController.calculate(currentTicks, targetTicks);

        double output = power + ff;
        robot.turretMotor.setPower(Range.clip(output, -1.0, 1.0));
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
        return Math.toDegrees(Math.atan2((goalPose.getY(DistanceUnit.INCH)-turretcentY), (goalPose.getX(DistanceUnit.INCH)-turretcentX)));
        //FIXME OLD
        //return Math.toDegrees(Math.atan2((goalPose.getY(DistanceUnit.INCH)-robot.pinpoint.getPosY(DistanceUnit.INCH)), (goalPose.getX(DistanceUnit.INCH)-robot.pinpoint.getPosX(DistanceUnit.INCH))));
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
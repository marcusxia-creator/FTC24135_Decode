package org.firstinspires.ftc.teamcode.TeleOps;

import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.blueFarGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redCloseGoalPose;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.redFarGoalPose;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.LUT;
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

    private final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    private final double angleToTick = 1 / tickToAngle;

    public static double kP = 17, kI = 0, kD = 0.005, kS = 0.2, kV = 2; // turret motor drive pidcontroller
    public static double kP_motor = 20, kI_motor = 0, kD_motor = 0.005, kF = 2; // turret motor pidf

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


    public void initTurret() {
        robot.turretMotor.setTargetPosition(0);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
    }

    public void resetTurretPosition() {
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public Turret (RobotHardware robot, boolean isRedAlliance) {
        this.robot = robot;
        pidController = new PIDController(kP, kI, kD);
        //this.limelight = limelight;
        this.robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        if (isRedAlliance) {
            targetPose = redTargetPose;
        }
        if (!isRedAlliance) {
            targetPose = blueTargetPose;
        }
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
            PIDFCoefficients pidf = new PIDFCoefficients(kP_motor, kI_motor, kD_motor, kF);
            robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            lastkPmotor = kP_motor; lastkImotor = kI_motor; lastkDmotor = kD_motor; lastkF = kF;
        }
    }


    public double getTurretDriveAngle () {
        return -(floorMod(robot.pinpoint.getHeading(AngleUnit.DEGREES) - getTargetAngle()+180, 360)-180);
        ///  Added the correction Angle from Limelight
        //double correctX = limelight.getTargetX();
        //double rawAngle = -(floorMod(robot.pinpoint.getHeading(AngleUnit.DEGREES) - getTargetAngle()+180, 360)-180);
        //return rawAngle - correctX;
    }

    public double getTurretMotorAngle(){
        return (robot.turretMotor.getCurrentPosition() * tickToAngle);
    }

    public void driveTurretMotor(){
        //updatePidFromDashboard();
        int ticks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(1);
    }

    public void driveTurretPID() {
        int targetTicks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        int currentTicks = robot.turretMotor.getCurrentPosition();
        double ff = (kS * Math.signum(targetTicks)) + (kV * targetTicks);
        double power = pidController.calculate(robot.turretMotor.getCurrentPosition(), targetTicks);
        double output = power + ff;
        robot.turretMotor.setPower(Range.clip(output, -1.0, 1.0));
    }

    public int getTargetTick () {
        return (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
    }

    public int getCurrentTick () {
        return robot.turretMotor.getCurrentPosition();
    }

    /*public boolean isTurretAtPosition (){
        if (Math.floor(angle) == Math.floor(getTurretMotorAngle())){
            return true;
        }
        else {
            return false;
        }
    }
     */

    public double getTargetAngle () {
        return Math.toDegrees(Math.atan2((goalPose.getY(DistanceUnit.INCH)-robot.pinpoint.getPosY(DistanceUnit.INCH)), (goalPose.getX(DistanceUnit.INCH)-robot.pinpoint.getPosX(DistanceUnit.INCH))));
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
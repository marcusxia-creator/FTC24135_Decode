package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.apache.commons.math3.filter.KalmanFilter;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Turret {
    /*We will control the turret using input from the pinpoint
    turret is continuously running and is separate from the shooter control
     */

    private final RobotHardware robot;

    private final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    private final double angleToTick = 1 / tickToAngle;

    public static double kP = 17, kI = 0, kD = 0.005, kS = 0.2, kV = 2; // turret motor drive pidcontroller
    public static double kP_motor = 17, kI_motor = 0, kD_motor = 0.005, kF = 2; // turret motor pidf

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

    public Turret (RobotHardware robot) {
        this.robot = robot;
        pidController = new PIDController(kP, kI, kD);
        //this.limelight = limelight;
        this.robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
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
        return Math.toDegrees(Math.atan2((62-robot.pinpoint.getPosY(DistanceUnit.INCH)), (-66-robot.pinpoint.getPosX(DistanceUnit.INCH))));
    }

    private double floorMod(double x, double y){
        return x-(Math.floor(x/y) * y);
    }
}
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

    public static double kP = 17, kI = 0, kD = 0.005, kS = 0.2, kV = 2;

    private PIDController pidController;
    private Limelight limelight;
    PIDFCoefficients pidf = new PIDFCoefficients(
            kP,      // P
            kI,      // I
            kD,      // D
            kV       // F
    );

    public Turret (RobotHardware robot) {
        this.robot = robot;
        pidController = new PIDController(kP, kI, kD);
        //this.limelight = limelight;
        this.robot.turretMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    public int motorDriveTick() {
        return (int) Math.round(getTurretDriveAngle() * angleToTick);
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
        return Math.toDegrees(Math.atan2((66-robot.pinpoint.getPosY(DistanceUnit.INCH)), (-66-robot.pinpoint.getPosX(DistanceUnit.INCH))));
    }

    private double floorMod(double x, double y){
        return x-(Math.floor(x/y) * y);
    }
}
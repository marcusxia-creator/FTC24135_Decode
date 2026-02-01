package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Turret {
    /*We will control the turret using input from the pinpoint
    turret is continuously running and is separate from the shooter control
     */

    private final RobotHardware robot;

    private final double tickToAngle = ((0.16867469879518 * 360) / 145.1);
    private final double angleToTick = 1 / tickToAngle;
    public int ticks;

    public Turret (RobotHardware robot) {
        this.robot = robot;
    }

    public int motorDriveTick() {
        return (int) Math.round(getTurretDriveAngle() * angleToTick);
    }

    public double getTurretDriveAngle () {
        return -(floorMod(robot.pinpoint.getHeading(AngleUnit.DEGREES) - getTargetAngle()+180, 360)-180);
    }

    public double getTurretMotorAngle(){
        return (robot.turretMotor.getCurrentPosition() * tickToAngle);
    }

    public void driveTurretMotor(){
        ticks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(0.8);
    }

    public void driveTurretPID() {
        ticks = (int)(Range.clip(getTurretDriveAngle(), -180, 180) * angleToTick);
        robot.turretMotor.setTargetPosition(ticks);
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.turretMotor.setPower(0.8);
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
        return Math.toDegrees(Math.atan2((72-robot.pinpoint.getPosY(DistanceUnit.INCH)), (-72-robot.pinpoint.getPosX(DistanceUnit.INCH))));
    }

    private double floorMod(double x, double y){
        return x-(Math.floor(x/y) * y);
    }
}
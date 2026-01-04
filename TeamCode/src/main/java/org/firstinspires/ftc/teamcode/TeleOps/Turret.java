package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Turret {
    /*We will control the turret using input from the pinpoint
    turret is continuously running and is separate from the shooter control
     */

    private final RobotHardware robot;

    private final double tickToAngle = 145.1 * 0.16867469879518 * 360;
    private final double angleToTick = 1 / tickToAngle;

    public Turret (RobotHardware robot) {
        this.robot = robot;
    }

    public int motorDriveTick() {
        return (int) Math.round(getTurretDriveAngle() / angleToTick);
    }

    public double getTurretDriveAngle () {
        if (robot.pinpoint.getHeading(AngleUnit.DEGREES) > 0) {
            return 90 - robot.pinpoint.getHeading(AngleUnit.DEGREES)  + getTargetAngle();
        }
        if (robot.pinpoint.getHeading(AngleUnit.DEGREES) < 0) {
            return 180 + robot.pinpoint.getHeading(AngleUnit.DEGREES) + getTargetAngle();
        }

        return 90 + getTargetAngle();
    }

    public double getTurretAngle() {
        return (robot.pinpoint.getHeading(AngleUnit.DEGREES) + getTurretMotorAngle());
    }

    public double getTurretMotorAngle(){
        return (robot.turretMotor.getCurrentPosition() / tickToAngle);
    }

    public double getTargetAngle () {
        return (180 - Math.atan(robot.pinpoint.getPosY(DistanceUnit.INCH) / robot.pinpoint.getPosX(DistanceUnit.INCH)));
    }

}

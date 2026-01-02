package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Turret {
    /*We will control the turret using input from the pinpoint
    turret is continuously running and is separate from the shooter control
     */

    private final RobotHardware robot;

    public Turret (RobotHardware robot) {
        this.robot = robot;
    }

    public int getTargetTick () {
        return 200;
    }

    public double getTargetAngle () {
        return (180 - Math.atan(robot.pinpoint.getPosY(DistanceUnit.INCH) / robot.pinpoint.getPosX(DistanceUnit.INCH)));
    }

}

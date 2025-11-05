package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ShooterPowerCalculator {

    private RobotHardware robot;

    private double distance;

    private Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    private Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45);

    public ShooterPowerCalculator (RobotHardware robot) {
        this.robot = robot;
    }

    public double getPower() {
        robot.pinpoint.update();
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - redGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - redGoalPose.getY(DistanceUnit.INCH), 2));
        //return a * Math.pow(distance, exp);
        return 0.9;
    }
}

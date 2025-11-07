package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class ShooterPowerCalculator {

    private RobotHardware robot;

    private double distance;

    public final Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    public final Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, 70, 70, AngleUnit.DEGREES, -45);

    public Pose2D goalPose;

    public ShooterPowerCalculator (RobotHardware robot) {
        this.robot = robot;
    }

    public void setGoal(Pose2D goalPose){
        this.goalPose = goalPose;
    }

    public double getDistance() {
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - goalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - goalPose.getY(DistanceUnit.INCH), 2));
        return distance;
    }

    public double getAngle(){
        return Math.atan2(robot.pinpoint.getPosY(DistanceUnit.METER)-goalPose.getY(DistanceUnit.METER),robot.pinpoint.getPosX(DistanceUnit.METER)-goalPose.getX(DistanceUnit.METER));
    }

    public double getPower() {
        robot.pinpoint.update();
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - goalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - goalPose.getY(DistanceUnit.INCH), 2));
        //return a * Math.pow(distance, exp);
        return 0.9;
    }
}

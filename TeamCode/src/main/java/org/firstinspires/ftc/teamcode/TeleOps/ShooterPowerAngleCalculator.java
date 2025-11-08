package org.firstinspires.ftc.teamcode.TeleOps;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import static org.firstinspires.ftc.teamcode.TeleOps.RobotActionConfig.*;

public class ShooterPowerAngleCalculator {

    private RobotHardware robot;

    private double distance;

    private final Pose2D redGoalPose = new Pose2D(DistanceUnit.INCH, -70, 70, AngleUnit.DEGREES, -45);
    private final Pose2D blueGoalPose = new Pose2D(DistanceUnit.INCH, -70, -70, AngleUnit.DEGREES, -45);

    private Pose2D actualGoalPose = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0);


    public ShooterPowerAngleCalculator(RobotHardware robot) {
        this.robot = robot;
    }

    public void setAlliance (boolean isRedAlliance) {
        if (isRedAlliance) {
            actualGoalPose = redGoalPose;
        }

        if (!isRedAlliance) {
            actualGoalPose = blueGoalPose;
        }
    }

    public double getDistance() {
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH), 2));
        return distance;
    }

    public double getPower() {
        distance = Math.sqrt(Math.pow(robot.pinpoint.getPosX(DistanceUnit.INCH) - actualGoalPose.getX(DistanceUnit.INCH),2) + Math.pow(robot.pinpoint.getPosY(DistanceUnit.INCH) - actualGoalPose.getY(DistanceUnit.INCH), 2));
        return a*Math.pow(distance, 2) + b*distance + c + correction;
        //return 0.9;
    }

    public double getAngle() {
        double tangent = (actualGoalPose.getY(DistanceUnit.INCH) - robot.pinpoint.getPosY(DistanceUnit.INCH)) / distance;
        return Math.toDegrees(Math.atan(tangent));
    }
}
